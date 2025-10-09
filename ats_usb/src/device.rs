use anyhow::{anyhow, Context, Result};
use protodongers::{PocMarkersReport, VendorData};
use std::{
    any::Any,
    borrow::Cow,
    net::TcpStream,
    pin::Pin,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex, Weak,
    },
    task::Poll,
    time::Duration,
};
use tokio::{
    io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt}, net::{ToSocketAddrs, UdpSocket, lookup_host}, sync::{mpsc, oneshot}, time::sleep
};
use tokio_stream::{wrappers::ReceiverStream, Stream, StreamExt};
use tracing::{debug, error, info, instrument, trace, warn};
use num_derive::{FromPrimitive, ToPrimitive};
use nusb::{DeviceInfo, io::{EndpointRead, EndpointWrite}, transfer::{Bulk, In, Out}};

use crate::{
    packets::vm::{
        AccelReport, CombinedMarkersReport, GeneralConfig, ImpactReport, MotData, ObjectReport,
        Packet, PacketData, PacketType, Port, Props, Register, StreamUpdate, WriteRegister,
    },
};

pub const COBS_DELIMITER: u8 = 0x00;

#[derive(Default)]
enum ResponseChannel {
    #[default]
    None,
    Oneshot(oneshot::Sender<PacketData>),
    Stream(mpsc::Sender<PacketData>),
}

#[derive(PartialEq, Eq, FromPrimitive, ToPrimitive)]
pub enum ProductId {
    PajUsb = 0x520F,
    PajAts = 0x5210,
    PocAts = 0x5211,
}

struct State {
    // id 255 is reserved for requests that don't care for a response
    response_channels: Mutex<[ResponseChannel; 255]>,
    streams_active: StreamsActive,
}

/// A helper struct to deal with cancellation
struct ResponseSlot {
    thread_state: Weak<State>,
    id: u8,
    finished: bool,
}

impl Drop for ResponseSlot {
    fn drop(&mut self) {
        // If the future holding the slot was cancelled, remove the sender from the
        // channel.
        if !self.finished {
            if let Some(thread_state) = self.thread_state.upgrade() {
                thread_state.response_channels.lock().unwrap()[usize::from(self.id)] =
                    ResponseChannel::None;
            }
        }
    }
}

#[derive(Debug)]
struct StreamsActive([AtomicBool; 256]);

impl Default for StreamsActive {
    fn default() -> Self {
        StreamsActive(std::array::from_fn(|_| AtomicBool::new(false)))
    }
}

impl std::ops::Index<PacketType> for StreamsActive {
    type Output = AtomicBool;

    fn index(&self, index: PacketType) -> &Self::Output {
        &self.0[u8::from(index) as usize]
    }
}

impl std::ops::IndexMut<PacketType> for StreamsActive {
    fn index_mut(&mut self, index: PacketType) -> &mut Self::Output {
        &mut self.0[u8::from(index) as usize]
    }
}

pub struct PacketTransport {
    writer: mpsc::Sender<Packet>,
    incoming_rx: mpsc::Receiver<Packet>,
}

#[derive(Clone)]
pub struct PacketTransportTx {
    writer: mpsc::Sender<Packet>,
}

impl PacketTransportTx {
    #[inline]
    pub async fn send(&self, pkt: Packet) -> Result<(), mpsc::error::SendError<Packet>> {
        self.writer.send(pkt).await
    }
}

impl PacketTransport {
    pub fn usb(mut in_ep: EndpointRead<Bulk>, mut out_ep: EndpointWrite<Bulk>) -> Self {
        let (writer, mut writer_rx) = mpsc::channel::<Packet>(64);
        let (incoming_tx, incoming_rx) = mpsc::channel::<Packet>(128);

        tokio::spawn(async move {
            let mut raw = Vec::with_capacity(1024);
            while let Some(pkt) = writer_rx.recv().await {
                raw.clear();
                if let Err(e) = postcard::to_io(&pkt, &mut raw) {
                    error!("postcard serialize failed: {e}");
                    continue;
                }
                if let Err(e) = out_ep.write_all(&raw).await {
                    error!("usb write_all failed: {e}");
                    continue;
                }
                if let Err(e) = out_ep.flush_end_async().await {
                    error!("usb flush_end failed: {e:?}");
                    continue;
                }
            }
            info!("usb writer exits");
        });

        tokio::spawn(async move {
            let mut reader = in_ep.until_short_packet();
            let mut io_errs = 0u8;
            loop {
                let mut buf = Vec::with_capacity(256);
                match reader.read_to_end(&mut buf).await {
                    Err(e) => {
                        use std::io::ErrorKind::*;
                        match e.kind() {
                            TimedOut | WouldBlock => { io_errs = 0; continue; }
                            BrokenPipe | UnexpectedEof | ConnectionReset => break,
                            _ => {
                                if io_errs < 3 { warn!("usb read error (ignored): {e}"); io_errs += 1; continue; }
                                error!("usb read error: {e}"); break;
                            }
                        }
                    }
                    Ok(_) => { io_errs = 0; }
                }
                if let Err(e) = reader.consume_end() {
                    warn!("usb consume_end failed: {e:?}");
                    continue;
                }
                match postcard::from_bytes::<Packet>(&buf) {
                    Ok(pkt) => {
                        if incoming_tx.send(pkt).await.is_err() { break; }
                    }
                    Err(e) => error!("postcard decode failed: {e:?}"),
                }
            }
            info!("usb reader exits");
        });

        PacketTransport { writer, incoming_rx }
    }
        
    pub async fn send(&self, pkt: Packet) -> Result<(), tokio::sync::mpsc::error::SendError<Packet>> {
        self.writer.send(pkt).await
    }

    pub fn split(self) -> (PacketTransportTx, ReceiverStream<Packet>) {
        let PacketTransport { writer, incoming_rx } = self;
        let tx_only = PacketTransportTx { writer };
        (tx_only, ReceiverStream::new(incoming_rx))
    }
}

#[derive(Clone)]
pub struct VmDevice {
    transport: PacketTransportTx,
    thread_state: Weak<State>,
}

impl VmDevice {
    pub fn from_transport(transport: PacketTransport) -> Self {
        let (tx_only, mut incoming) = transport.split();

        let response_channels = std::array::from_fn(|_| ResponseChannel::None);
        let state = Arc::new(State {
            response_channels: Mutex::new(response_channels),
            streams_active: StreamsActive::default(),
        });
        let thread_state = Arc::downgrade(&state);
        let state_cloned = Arc::clone(&state);

        tokio::spawn(async move {
            use tokio_stream::StreamExt;
            while let Some(reply) = incoming.next().await {
                let mut chans = state_cloned.response_channels.lock().unwrap();
                let slot = &mut chans[usize::from(reply.id)];
                let data = reply.data;

                let err = match std::mem::take(slot) {
                    ResponseChannel::None => None,
                    ResponseChannel::Oneshot(tx) => tx.send(data).err().map(|e| format!("{e:?}")),
                    ResponseChannel::Stream(tx) => {
                        let e = tx.try_send(data).err().map(|e| format!("{e:?}"));
                        *slot = ResponseChannel::Stream(tx);
                        e
                    }
                };
                if let Some(e) = err {
                    debug!("dispatcher failed to send reply: {e}");
                }
            }
            info!("dispatcher exits");
        });

        VmDevice { transport: tx_only, thread_state }
    }

    pub async fn connect_usb(info: DeviceInfo) -> Result<Self> {
        let dev = info.open().await?;
        let iface = dev.claim_interface(0).await?;

        let in_ep  = iface.endpoint::<Bulk, In >(0x81)?.reader(4096);
        let out_ep = iface.endpoint::<Bulk, Out>(0x01)?.writer(4096);

        let transport = PacketTransport::usb(in_ep, out_ep);
        Ok(Self::from_transport(transport))
    }

    pub async fn send(&self, pkt: Packet) -> Result<(), mpsc::error::SendError<Packet>> {
        self.transport.send(pkt).await
    }

    pub async fn request(&self, data: PacketData) -> anyhow::Result<PacketData> {
        let (mut slot, recv) = self.get_oneshot_slot()?;
        self.send(Packet { id: slot.id, data }).await?;
        let r = recv.await?;
        slot.finished = true;
        Ok(r)
    }

    fn get_oneshot_slot(
        &self,
    ) -> anyhow::Result<(ResponseSlot, oneshot::Receiver<PacketData>)> {
        if let Some(thread_state) = self.thread_state.upgrade() {
            let mut response_channels = thread_state.response_channels.lock().unwrap();
            for (i, c) in response_channels.iter_mut().enumerate() {
                if let ResponseChannel::None = c {
                    let (send, receiver) = oneshot::channel();
                    *c = ResponseChannel::Oneshot(send);
                    return Ok((
                        ResponseSlot {
                            thread_state: self.thread_state.clone(),
                            id: i as u8,
                            finished: false,
                        },
                        receiver,
                    ));
                }
            }
        }
        Err(anyhow::anyhow!("Failed to allocate request id"))
    }

    fn get_stream_slot(
        &self,
        buffer: usize,
    ) -> Result<(ResponseSlot, mpsc::Receiver<PacketData>), anyhow::Error> {
        if let Some(thread_state) = self.thread_state.upgrade() {
            let mut response_channels = thread_state.response_channels.lock().unwrap();
            for (i, c) in response_channels.iter_mut().enumerate() {
                if let ResponseChannel::None = c {
                    let (send, receiver) = mpsc::channel(buffer);
                    *c = ResponseChannel::Stream(send);
                    return Ok((
                        ResponseSlot {
                            thread_state: self.thread_state.clone(),
                            id: i as u8,
                            finished: false,
                        },
                        receiver,
                    ));
                }
            }
        }
        Err(anyhow::anyhow!("Failed to allocate request id"))
    }

    #[instrument(skip(self))]
    pub async fn read_register(&self, port: Port, bank: u8, address: u8) -> Result<u8> {
        let r = self
            .request(PacketData::ReadRegister(Register {
                port,
                bank,
                address,
            }))
            .await?
            .read_register_response()
            .with_context(|| "unexpected response")?;
        info!(data = r.data);
        assert_eq!(r.bank, bank);
        assert_eq!(r.address, address);
        Ok(r.data)
    }

    pub async fn write_register(&self, port: Port, bank: u8, address: u8, data: u8) -> Result<()> {
        let data = PacketData::WriteRegister(WriteRegister {
            port,
            bank,
            address,
            data,
        });
        let pkt = Packet { id: 255, data };
        self.transport.writer.send(pkt).await?;
        Ok(())
    }

    pub async fn write_vendor(&self, tag: u8, data: &[u8]) -> Result<()> {
        assert!(tag > PacketType::VendorStart().into() && tag < PacketType::VendorEnd().into());
        let data_len = data.len();
        let data_padded: [u8; 98] = {
            let mut padded = [0; 98];
            padded[..data_len].copy_from_slice(data);
            padded
        };
        let data = PacketData::Vendor(
            tag,
            VendorData {
                len: data_len as u8,
                data: data_padded,
            },
        );
        let pkt = Packet { id: 255, data };
        self.transport.writer.send(pkt).await?;
        Ok(())
    }

    pub async fn read_config(&self) -> Result<GeneralConfig> {
        let r = self
            .request(PacketData::ReadConfig())
            .await?
            .read_config_response()
            .with_context(|| "unexpected response")?;
        info!("config: {:?}", r);
        Ok(r)
    }

    pub async fn write_config(&self, config: GeneralConfig) -> Result<()> {
        let data = PacketData::WriteConfig(config);
        let pkt = Packet { id: 255, data };
        self.transport.writer.send(pkt).await?;
        Ok(())
    }

    pub async fn read_props(&self) -> Result<Props> {
        let r = self
            .request(PacketData::ReadProps())
            .await?
            .read_props_response()
            .with_context(|| "unexpected response")?;
        info!("props: {:?}", r);
        Ok(r)
    }

    pub async fn get_frame(&self) -> Result<([MotData; 16], [MotData; 16])> {
        let r = self
            .request(PacketData::ObjectReportRequest())
            .await?
            .object_report()
            .with_context(|| "unexpected response")?;
        Ok((r.mot_data_nf, r.mot_data_wf))
    }

    pub async fn stream(
        &self,
        stream_type: PacketType,
    ) -> Result<impl Stream<Item = PacketData> + Send + Sync> {
        if let Some(thread_state) = self.thread_state.upgrade() {
            if thread_state.streams_active[stream_type].swap(true, Ordering::Relaxed) {
                return Err(anyhow!("cannot have more than one {stream_type:?} stream"));
            }
            let (slot, receiver) = self.get_stream_slot(100)?;
            self.transport.writer
                .send(Packet {
                    id: slot.id,
                    data: PacketData::StreamUpdate(StreamUpdate {
                        packet_id: stream_type,
                        action: crate::packets::vm::StreamUpdateAction::Enable,
                    }),
                })
                .await?;
            return Ok(PacketStream {
                slot,
                receiver: ReceiverStream::new(receiver),
                sender: self.transport.writer.clone(),
                stream_type,
            });
        }
        Err(anyhow!("thread state dropped"))
    }

    pub async fn stream_mot_data(&self) -> Result<impl Stream<Item = ObjectReport> + Send + Sync> {
        Ok(self
            .stream(PacketType::ObjectReport())
            .await?
            .filter_map(|x| x.object_report()))
    }

    pub async fn stream_combined_markers(
        &self,
    ) -> Result<impl Stream<Item = CombinedMarkersReport> + Send + Sync> {
        Ok(self
            .stream(PacketType::CombinedMarkersReport())
            .await?
            .filter_map(|x| x.combined_markers_report()))
    }

    pub async fn stream_poc_markers(
        &self,
    ) -> Result<impl Stream<Item = PocMarkersReport> + Send + Sync> {
        Ok(self
            .stream(PacketType::PocMarkersReport())
            .await?
            .filter_map(|x| x.poc_markers_report()))
    }

    pub async fn stream_accel(&self) -> Result<impl Stream<Item = AccelReport> + Send + Sync> {
        Ok(self
            .stream(PacketType::AccelReport())
            .await?
            .filter_map(|x| x.accel_report()))
    }

    pub async fn stream_impact(&self) -> Result<impl Stream<Item = ImpactReport> + Send + Sync> {
        Ok(self
            .stream(PacketType::ImpactReport())
            .await?
            .filter_map(|x| x.impact_report()))
    }

    pub async fn flash_settings(&self) -> Result<()> {
        self.transport.writer
            .send(Packet {
                id: 255,
                data: PacketData::FlashSettings(),
            })
            .await?;
        Ok(())
    }

    pub async fn write_mode(&self, mode: protodongers::Mode) -> Result<()> {
        let data = PacketData::WriteMode(mode);
        let pkt = Packet { id: 255, data };
        self.transport.writer.send(pkt).await?;
        Ok(())
    }
}

pub struct PacketStream {
    stream_type: PacketType,
    slot: ResponseSlot,
    sender: mpsc::Sender<Packet>,
    receiver: ReceiverStream<PacketData>,
}

impl Stream for PacketStream {
    type Item = PacketData;

    fn poll_next(
        self: Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> Poll<Option<Self::Item>> {
        Pin::new(&mut self.get_mut().receiver).poll_next(cx)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.receiver.size_hint()
    }
}

impl Drop for PacketStream {
    fn drop(&mut self) {
        if let Some(thread_state) = self.slot.thread_state.upgrade() {
            thread_state.streams_active[self.stream_type].store(false, Ordering::Relaxed);
            let _ = self
                .sender
                .try_send(Packet {
                    id: 255,
                    data: PacketData::StreamUpdate(StreamUpdate {
                        packet_id: self.stream_type,
                        action: crate::packets::vm::StreamUpdateAction::Disable,
                    }),
                })
                .inspect_err(|e| warn!("Failed to stop stream: {e}"));
        }
    }
}

macro_rules! read_register_spec {
    ($name:ident : $ty:ty = $bank:literal; [$($addr:literal),*]) => {
        pub async fn $name(&self, port: Port) -> ::anyhow::Result<$ty> {
            let mut bytes = <$ty>::to_le_bytes(0);
            for (byte, addr) in ::std::iter::zip(&mut bytes, [$($addr),*]) {
                *byte = self.read_register(port, $bank, addr).await?;
            }
            Ok(<$ty>::from_le_bytes(bytes))
        }
    }
}

macro_rules! write_register_spec {
    ($name:ident : $ty:ty = $bank:literal; [$($addr:literal),*]) => {
        pub async fn $name(&self, port: Port, value: $ty) -> ::anyhow::Result<()> {
            let bytes = <$ty>::to_le_bytes(value);
            for (byte, addr) in ::std::iter::zip(bytes, [$($addr),*]) {
                self.write_register(port, $bank, addr, byte).await?;
            }
            Ok(())
        }
    }
}

impl VmDevice {
    // paj
    read_register_spec!(product_id: u16 = 0x00; [0x02, 0x03]);
    read_register_spec!(resolution_x: u16 = 0x0c; [0x60, 0x61]);
    read_register_spec!(resolution_y: u16 = 0x0c; [0x62, 0x63]);
    write_register_spec!(set_resolution_x: u16 = 0x0c; [0x60, 0x61]);
    write_register_spec!(set_resolution_y: u16 = 0x0c; [0x62, 0x63]);
    read_register_spec!(gain_1: u8 = 0x01; [0x05]); // B_global
    read_register_spec!(gain_2: u8 = 0x01; [0x06]); // B_ggh
    write_register_spec!(set_gain_1: u8 = 0x0c; [0x0b]); // B_global
    write_register_spec!(set_gain_2: u8 = 0x0c; [0x0c]); // B_ggh
    read_register_spec!(exposure_time: u16 = 0x01; [0x0e, 0x0f]);
    write_register_spec!(set_exposure_time: u16 = 0x0c; [0x0f, 0x10]);
    read_register_spec!(brightness_threshold: u8 = 0x0c; [0x47]);
    write_register_spec!(set_brightness_threshold: u8 = 0x0c; [0x47]);
    read_register_spec!(noise_threshold: u8 = 0x00; [0x0f]);
    write_register_spec!(set_noise_threshold: u8 = 0x00; [0x0f]);
    read_register_spec!(area_threshold_max: u16 = 0x00; [0x0b, 0x0c]);
    write_register_spec!(set_area_threshold_max: u16 = 0x00; [0x0b, 0x0c]);
    read_register_spec!(area_threshold_min: u8 = 0x0c; [0x46]);
    write_register_spec!(set_area_threshold_min: u8 = 0x0c; [0x46]);
    read_register_spec!(operation_mode: u8 = 0x00; [0x12]);
    write_register_spec!(set_operation_mode: u8 = 0x00; [0x12]);
    read_register_spec!(max_object_cnt: u8 = 0x00; [0x19]);
    write_register_spec!(set_max_object_cnt: u8 = 0x00; [0x19]);
    read_register_spec!(frame_subtraction: u8 = 0x00; [0x28]);
    write_register_spec!(set_frame_subtraction: u8 = 0x00; [0x28]);
    read_register_spec!(frame_period: u32 = 0x0c; [0x07, 0x08, 0x09]);
    write_register_spec!(set_frame_period: u32 = 0x0c; [0x07, 0x08, 0x09]);
    write_register_spec!(set_bank1_sync_updated: u8 = 0x01; [0x01]);
    write_register_spec!(set_bank0_sync_updated: u8 = 0x00; [0x01]);

    // pag
    read_register_spec!(pag_chip_id: u16 = 0x00; [0x00, 0x01]);
    read_register_spec!(pag_fps: u16 = 0x00; [0x13]);
    write_register_spec!(set_pag_fps: u8 = 0x00; [0x13]);
    read_register_spec!(pag_exposure: u8 = 0x00; [0x66]);
    write_register_spec!(set_pag_exposure: u8 = 0x00; [0x66]);
    read_register_spec!(pag_gain: u8 = 0x00; [0x67]);
    write_register_spec!(set_pag_gain: u8 = 0x00; [0x67]);
    read_register_spec!(pag_area_lower: u16 = 0x00; [0x68, 0x69]);
    write_register_spec!(set_pag_area_lower: u16 = 0x00; [0x68, 0x69]);
    read_register_spec!(pag_area_upper: u16 = 0x00; [0x6A, 0x6B]);
    write_register_spec!(set_pag_area_upper: u16 = 0x00; [0x6A, 0x6B]);
    read_register_spec!(pag_light_threshold: u8 = 0x00; [0x6C]);
    write_register_spec!(set_pag_light_threshold: u8 = 0x00; [0x6C]);
}
