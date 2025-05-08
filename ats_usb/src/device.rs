use anyhow::{anyhow, Context, Result};
use protodongers::PocMarkersReport;
use serial2;
use std::{
    any::Any,
    borrow::Cow,
    io::{BufRead, BufReader, ErrorKind, Read, Write},
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
    net::{lookup_host, ToSocketAddrs, UdpSocket},
    sync::{mpsc, oneshot},
    time::sleep,
};
use tokio_stream::{wrappers::ReceiverStream, Stream, StreamExt};
use tracing::{debug, error, info, instrument, trace, warn};

use num_derive::{FromPrimitive, ToPrimitive};

use crate::{
    packets::vm::{
        AccelReport, CombinedMarkersReport, GeneralConfig, ImpactReport, MotData, ObjectReport,
        Packet, PacketData, PacketType, Port, Props, Register, StreamUpdate, WriteRegister,
    },
    udp_stream::UdpStream,
};

pub const SLIP_FRAME_END: u8 = 0xc0;
const SLIP_FRAME_ESC: u8 = 0xdb;
const SLIP_FRAME_ESC_END: u8 = 0xdc;
const SLIP_FRAME_ESC_ESC: u8 = 0xdd;

#[derive(Default)]
enum ResponseChannel {
    #[default]
    None,
    Oneshot(oneshot::Sender<PacketData>),
    Stream(mpsc::Sender<PacketData>),
}

#[derive(Clone, Debug)]
pub struct UsbDevice {
    to_thread: mpsc::Sender<Packet>,
    thread_state: Weak<State>,
}

#[derive(PartialEq, Eq)]
#[derive(FromPrimitive, ToPrimitive)]
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

impl UsbDevice {
    /// Connect to the device using the serial port at `path`. Starts two background threads to
    /// service reads and writes.
    pub async fn connect_serial<'a>(path: impl Into<Cow<'a, str>>, wait_dsr: bool) -> Result<Self> {
        let path = path.into();
        let path_copy = path.to_string();
        info!("Connecting to {path}...");
        let read_port_future = tokio::task::spawn_blocking(move || {
            serial2::SerialPort::open(&path_copy, |mut settings: serial2::Settings| {
                settings.set_raw();
                settings.set_baud_rate(115200)?;
                settings.set_char_size(serial2::CharSize::Bits7);
                settings.set_stop_bits(serial2::StopBits::Two);
                settings.set_parity(serial2::Parity::Odd);
                settings.set_flow_control(serial2::FlowControl::RtsCts);
                Ok(settings)
            })
        });
        let mut read_port = tokio::time::timeout(Duration::from_secs(5), read_port_future)
            .await
            .with_context(|| format!("Failed to connect to {}: timed out after 5 seconds", path))?
            .unwrap()?;

        read_port.set_read_timeout(Duration::from_millis(300))?;
        read_port.set_dtr(true)?;

        let mut port = tokio::task::spawn_blocking(move || -> Result<_> {
            if wait_dsr {
                while read_port.read_dsr()? == false {
                    std::thread::sleep(Duration::from_millis(100));
                }
            }
            let mut buf = vec![0xff];
            Packet {
                id: 0,
                data: PacketData::StreamUpdate(StreamUpdate {
                    packet_id: PacketType::End(),
                    action: crate::packets::vm::StreamUpdateAction::DisableAll,
                }),
            }
            .serialize(&mut buf);
            read_port.write_all(&buf).unwrap();
            let mut drained = 0;
            loop {
                let Ok(bytes_read) = read_port.read(&mut [0; 1024]) else {
                    break;
                };
                drained += bytes_read;
            }
            if drained > 0 {
                info!("{drained} bytes drained");
            }
            Ok(read_port)
        })
        .await??;

        let writer = port.try_clone().context("Failed to clone serial port")?;
        port.set_read_timeout(Duration::from_millis(3000))?;
        let reader = port;
        Ok(Self::new(reader, writer, wait_dsr))
    }

    pub fn connect_tcp(addr: &str) -> Result<Self> {
        info!("Connecting to {addr}...");
        let conn = TcpStream::connect(addr)?;
        conn.set_read_timeout(Some(Duration::from_millis(3000)))?;
        let conn2 = conn.try_clone().context("Failed to clone tcp stream")?;
        Ok(Self::new(conn, conn2, false))
    }

    pub async fn connect_hub(local_addr: impl ToSocketAddrs, device_addr: &str) -> Result<Self> {
        info!("Connecting to {device_addr}...");
        let sock = socket2::Socket::new(
            socket2::Domain::IPV4,
            socket2::Type::DGRAM,
            Some(socket2::Protocol::UDP),
        )?;
        sock.set_nonblocking(true)?;
        let addr = match lookup_host(local_addr).await?.next() {
            Some(addr) => addr,
            None => anyhow::bail!("Failed to resolve local address"),
        };
        sock.bind(&socket2::SockAddr::from(addr))?;
        let sock = UdpSocket::from_std(sock.into())?;
        sock.set_broadcast(true)?;
        sock.connect(device_addr).await?;
        sock.send(&[255, 1]).await?;
        match sock.recv(&mut []).await {
            Ok(_) => {}
            Err(e) => {
                if e.raw_os_error() != Some(10040) {
                    error!("Failed to receive response from device: {e}");
                    return Err(e.into());
                }
            }
        }

        // todo get rid of device 1 assumption
        let mut buf = vec![1, 0, 0xff];
        Packet {
            id: 0,
            data: PacketData::StreamUpdate(StreamUpdate {
                packet_id: PacketType::End(),
                action: crate::packets::vm::StreamUpdateAction::DisableAll,
            }),
        }
        .serialize(&mut buf);
        sock.send(&buf).await?;
        sleep(Duration::from_millis(100)).await;

        let sock = sock.into_std()?;
        sock.set_nonblocking(false)?;
        sock.set_read_timeout(Some(Duration::from_millis(3000)))?;
        let sock2 = sock.try_clone().context("Failed to clone udp socket")?;
        let read = UdpStream::with_capacity(sock, 1472, 0, &[1, 0], &[]);
        let write = UdpStream::with_capacity(sock2, 0, 1472, &[], &[1, 0]);
        Ok(Self::new(read, write, false))
    }

    pub fn new<R, W>(mut reader: R, writer: W, check_dsr: bool) -> Self
    where
        R: Any + Read + Send + 'static,
        W: Write + Send + 'static,
    {
        let response_channels = std::array::from_fn(|_| ResponseChannel::None);
        let mut state = Arc::new(State {
            response_channels: Mutex::new(response_channels),
            streams_active: StreamsActive::default(),
        });

        let thread_state = Arc::downgrade(&state);

        // Writer thread
        let (sender, mut receiver) = mpsc::channel::<Packet>(16);
        let mut writer = writer;
        std::thread::spawn(move || {
            let mut buf = vec![];
            loop {
                let Some(pkt) = receiver.blocking_recv() else {
                    break;
                };

                buf.clear();
                buf.push(0xff);
                pkt.serialize(&mut buf);

                debug!("write id={} len={} ty={:?}", pkt.id, buf.len(), pkt.ty());
                let r = writer.write_all(&buf).and_then(|_| writer.flush());
                if let Err(e) = r {
                    error!("device thread failed to write: {e}");
                }
            }
            info!("device writer thread exiting");
        });

        // Reader thread.
        std::thread::spawn(move || {
            let mut port = None;
            let mut _serialport;
            let mut reader: BufReader<&mut dyn Read> = if check_dsr {
                _serialport = (&reader as &dyn Any)
                    .downcast_ref::<serial2::SerialPort>()
                    .unwrap();
                port = Some(_serialport);
                BufReader::new(&mut _serialport)
            } else {
                BufReader::new(&mut reader)
            };
            let mut buf = vec![];
            let mut io_error_count = 0;
            loop {
                if Arc::get_mut(&mut state).is_some() {
                    info!("no more thread_state references");
                    break;
                }
                let reply: Result<_, std::io::Error> = (|| {
                    buf.clear();
                    let bytes_read = reader.read_until(SLIP_FRAME_END, &mut buf)?;
                    if bytes_read == 0 {
                        // eof
                        return Err(std::io::Error::new(ErrorKind::BrokenPipe, "disconnected"));
                    }
                    buf.pop();
                    if buf.contains(&SLIP_FRAME_ESC) {
                        match decode_slip_frame(&mut buf) {
                            Err(e) => return Ok(Err(e)),
                            Ok(_) => (),
                        };
                    }
                    trace!("read frame len={}", buf.len());
                    Ok(Packet::parse(&mut &buf[..])
                        .with_context(|| format!("failed to parse packet: {:?}", buf.clone()))
                        .inspect(|p| trace!("read id={} type={:?}", p.id, p.ty())))
                })();
                let reply = match reply {
                    // IO error
                    Err(e) => {
                        match e.kind() {
                            ErrorKind::TimedOut | ErrorKind::WouldBlock => {
                                debug!("read timed out");
                                io_error_count = 0;
                                if let Some(usb_device) = port {
                                    if let Ok(dsr) = usb_device.read_dsr() {
                                        if !dsr {
                                            info!("device disconnected");
                                            break;
                                        }
                                    } else {
                                        if io_error_count < 3 {
                                            warn!("error reading from device: {}, ignoring", e);
                                            io_error_count += 1;
                                            continue;
                                        }
                                        error!("error reading from device: {}", e);
                                        break;
                                    }
                                }
                                continue;
                            }
                            ErrorKind::BrokenPipe => {
                                info!("device disconnected");
                                break;
                            }
                            _ => (),
                        }
                        if io_error_count < 3 {
                            warn!("error reading from device: {}, ignoring", e);
                            io_error_count += 1;
                            continue;
                        }
                        error!("error reading from device: {}", e);
                        break;
                    }
                    Ok(r) => r,
                };
                io_error_count = 0;
                let reply = match reply {
                    // Parse error
                    Err(e) => {
                        error!("{:?}", e);
                        continue;
                    }
                    Ok(r) => r,
                };
                let mut response_channels = state.response_channels.lock().unwrap();
                let response_sender = &mut response_channels[usize::from(reply.id)];
                let e = match std::mem::take(response_sender) {
                    ResponseChannel::None => None,
                    ResponseChannel::Oneshot(oneshot) => {
                        oneshot.send(reply.data).err().map(|e| format!("{e:?}"))
                    }
                    ResponseChannel::Stream(sender) => {
                        let e = sender.try_send(reply.data).err().map(|e| format!("{e:?}"));
                        *response_sender = ResponseChannel::Stream(sender);
                        e
                    }
                };
                if let Some(e) = e {
                    debug!("device reader thread failed to send reply: {e:?}");
                }
            }
            info!("device reader thread exiting");
        });
        Self {
            to_thread: sender,
            thread_state,
        }
    }

    fn get_oneshot_slot(
        &self,
    ) -> Result<(ResponseSlot, oneshot::Receiver<PacketData>), anyhow::Error> {
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

    pub async fn request(&self, packet: PacketData) -> Result<PacketData> {
        let (mut response_slot, receiver) = self.get_oneshot_slot()?;
        self.to_thread
            .send(Packet {
                id: response_slot.id,
                data: packet,
            })
            .await?;
        let result = receiver.await;
        response_slot.finished = true;
        Ok(result?)
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
        self.to_thread.send(pkt).await?;
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
        let data = PacketData::Vendor(tag, (data_len as u8, data_padded));
        let pkt = Packet { id: 255, data };
        self.to_thread.send(pkt).await?;
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
        self.to_thread.send(pkt).await?;
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
            self.to_thread
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
                to_thread: self.to_thread.clone(),
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
        self.to_thread
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
        self.to_thread.send(pkt).await?;
        Ok(())
    }
}

pub fn encode_slip_frame(buf: &mut Vec<u8>) {
    let mut i = 0;
    while i < buf.len() {
        match buf[i] {
            SLIP_FRAME_END => {
                buf[i] = SLIP_FRAME_ESC;
                buf.insert(i + 1, SLIP_FRAME_ESC_END);
                i += 1;
            }
            SLIP_FRAME_ESC => {
                buf[i] = SLIP_FRAME_ESC;
                buf.insert(i + 1, SLIP_FRAME_ESC_ESC);
                i += 1;
            }
            x => {
                buf[i] = x;
            }
        }
        i += 1;
    }
    buf.push(SLIP_FRAME_END);
}

// TODO there's probably a faster SIMD way
pub fn decode_slip_frame(buf: &mut Vec<u8>) -> Result<()> {
    let mut j = 0;
    let mut esc = false;
    for i in 0..buf.len() {
        match buf[i] {
            self::SLIP_FRAME_ESC => {
                if esc {
                    anyhow::bail!("double esc");
                }
                esc = true;
            }
            self::SLIP_FRAME_ESC_END if esc => {
                buf[j] = SLIP_FRAME_END;
                j += 1;
                esc = false;
            }
            self::SLIP_FRAME_ESC_ESC if esc => {
                buf[j] = SLIP_FRAME_ESC;
                j += 1;
                esc = false;
            }
            x => {
                if esc {
                    anyhow::bail!("invalid esc");
                }
                buf[j] = x;
                j += 1;
            }
        }
    }
    if esc {
        anyhow::bail!("trailing esc");
    }
    buf.resize(j, 0);
    Ok(())
}

pub struct PacketStream {
    stream_type: PacketType,
    slot: ResponseSlot,
    to_thread: mpsc::Sender<Packet>,
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
                .to_thread
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

impl UsbDevice {
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

#[cfg(test)]
mod tests {
    #[test]
    fn test_decode_slip() {
        let mut slip_encoded = vec![0x01, 0xDB, 0xDC, 0xDB, 0xDD];
        super::decode_slip_frame(&mut slip_encoded).unwrap();
        assert_eq!([0x01, 0xC0, 0xDB], slip_encoded[..]);
    }
}
