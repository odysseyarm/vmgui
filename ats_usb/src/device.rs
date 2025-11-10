use anyhow::{anyhow, Context, Result};
use num_derive::{FromPrimitive, ToPrimitive};
use nusb::{
    io::{EndpointRead, EndpointWrite},
    transfer::{Bulk, ControlIn, ControlOut, ControlType, In, Out, Recipient},
    Device, DeviceInfo, Interface,
};
use protodongers::{
    control::usb_mux::{ClearBondsError, PairingError, UsbMuxCtrlMsg},
    PocMarkersReport, VendorData,
};
use std::{
    pin::Pin,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex, Weak,
    },
    task::Poll,
    time::Duration,
};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    sync::{mpsc, oneshot},
};
use tokio_stream::{wrappers::ReceiverStream, Stream, StreamExt};
use tracing::{debug, error, info, instrument, trace, warn};

use crate::packets::vm::{
    AccelConfig, AccelReport, CombinedMarkersReport, ConfigKind, GeneralConfig, GyroConfig,
    ImpactReport, MotData, ObjectReport, Packet, PacketData, PacketType, Port, Props, Register,
    StreamUpdate, WriteRegister,
};
use nalgebra::Isometry3;
use opencv_ros_camera::RosOpenCvIntrinsics;

fn default_camera_intrinsics() -> RosOpenCvIntrinsics<f32> {
    // Create a default camera with identity-like parameters
    // Using a simple pinhole camera model with no distortion
    use ats_common::ocv_types::{MinimalCameraCalibrationParams, OpenCVMatrix3, OpenCVMatrix5x1};
    MinimalCameraCalibrationParams {
        camera_matrix: OpenCVMatrix3 {
            data: [
                500.0, 0.0, 320.0, // fx, 0, cx
                0.0, 500.0, 240.0, // 0, fy, cy
                0.0, 0.0, 1.0, // 0, 0, 1
            ],
        },
        dist_coeffs: OpenCVMatrix5x1 {
            data: [0.0, 0.0, 0.0, 0.0, 0.0], // no distortion
        },
    }
    .into()
}

pub const COBS_DELIMITER: u8 = 0x00;

/// A struct that holds all configuration values together
/// This is needed because the new protodonge API returns individual config items as enum variants
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct GeneralSettings {
    pub impact_threshold: u8,
    pub suppress_ms: u8,
    pub accel_config: AccelConfig,
    pub gyro_config: GyroConfig,
    pub camera_model_nf: RosOpenCvIntrinsics<f32>,
    pub camera_model_wf: RosOpenCvIntrinsics<f32>,
    pub stereo_iso: Isometry3<f32>,
}

impl Default for GeneralSettings {
    fn default() -> Self {
        Self {
            impact_threshold: 100,
            suppress_ms: 100,
            accel_config: AccelConfig::default(),
            gyro_config: GyroConfig::default(),
            camera_model_nf: default_camera_intrinsics(),
            camera_model_wf: default_camera_intrinsics(),
            stereo_iso: Isometry3::identity(),
        }
    }
}

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
    cancel: Arc<tokio_util::sync::CancellationToken>,
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

        let cancel_token = tokio_util::sync::CancellationToken::new();

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
                            TimedOut | WouldBlock => {
                                io_errs = 0;
                                continue;
                            }
                            BrokenPipe | UnexpectedEof | ConnectionReset => break,
                            _ => {
                                if io_errs < 3 {
                                    warn!("usb read error (ignored): {e}");
                                    io_errs += 1;
                                    continue;
                                }
                                error!("usb read error: {e}");
                                break;
                            }
                        }
                    }
                    Ok(_) => {
                        io_errs = 0;
                    }
                }
                if let Err(e) = reader.consume_end() {
                    warn!("usb consume_end failed: {e:?}");
                    continue;
                }
                match postcard::from_bytes::<Packet>(&buf) {
                    Ok(pkt) => {
                        if incoming_tx.send(pkt).await.is_err() {
                            break;
                        }
                    }
                    Err(e) => error!("postcard decode failed: {e:?}"),
                }
            }
            info!("usb reader exits");
        });

        PacketTransport {
            writer,
            incoming_rx,
            cancel: Arc::new(cancel_token),
        }
    }

    /// Create a PacketTransport that routes VM packets through a MuxDevice
    pub fn mux(mux: MuxDevice, device_addr: [u8; 6]) -> Self {
        use rand::Rng;
        let transport_id: u32 = rand::thread_rng().gen();
        debug!(
            "PacketTransport::mux() [ID:{}] called for device {:02X}:{:02X}...",
            transport_id, device_addr[0], device_addr[1]
        );
        let (writer, mut writer_rx) = mpsc::channel::<Packet>(64);
        let (incoming_tx, incoming_rx) = mpsc::channel::<Packet>(128);

        let cancel_token = tokio_util::sync::CancellationToken::new();
        let cancel_reader = cancel_token.clone();

        let mux_clone = mux.clone();

        // Writer task: wraps outgoing VM packets in MuxMsg::SendTo
        let transport_id_writer = transport_id;
        tokio::spawn(async move {
            debug!("mux writer task started [ID:{}]", transport_id_writer);
            while let Some(pkt) = writer_rx.recv().await {
                info!("mux writer: sending packet to device {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                    device_addr[0], device_addr[1], device_addr[2], device_addr[3], device_addr[4], device_addr[5]);
                if let Err(e) = mux.send_to(device_addr, pkt).await {
                    error!("mux send_to failed: {e}");
                    break;
                }
            }
            debug!("mux writer task exits [ID:{}]", transport_id_writer);
        });

        // Reader task: unwraps incoming MuxMsg::DevicePacket for our device
        let transport_id_reader = transport_id;
        tokio::spawn(async move {
            debug!("mux reader task started [ID:{}]", transport_id_reader);
            loop {
                tokio::select! {
                    result = mux_clone.receive_msg() => {
                        match result {
                            Ok(msg) => {
                                // Only forward packets from our target device
                                if let crate::packets::mux::MuxMsg::DevicePacket(dev_pkt) = msg {
                                    if dev_pkt.dev == device_addr {
                                        debug!("mux reader: [ID:{}] forwarding packet", transport_id_reader);
                                        if incoming_tx.send(dev_pkt.pkt).await.is_err() {
                                            break;
                                        }
                                    } else {
                                        trace!("mux reader: ignoring packet from other device");
                                    }
                                } else {
                                    trace!("mux reader: ignoring non-DevicePacket message");
                                }
                            }
                            Err(e) => {
                                error!("mux receive_msg failed: {e}");
                                break;
                            }
                        }
                    }
                    _ = cancel_reader.cancelled() => {
                        debug!("mux reader: [ID:{}] cancelled, exiting", transport_id_reader);
                        break;
                    }
                }
            }
            debug!("mux reader task exits [ID:{}]", transport_id_reader);
        });

        PacketTransport {
            writer,
            incoming_rx,
            cancel: Arc::new(cancel_token),
        }
    }

    pub async fn send(
        &self,
        pkt: Packet,
    ) -> Result<(), tokio::sync::mpsc::error::SendError<Packet>> {
        self.writer.send(pkt).await
    }

    pub fn split(
        self,
    ) -> (
        PacketTransportTx,
        ReceiverStream<Packet>,
        Arc<tokio_util::sync::CancellationToken>,
    ) {
        let PacketTransport {
            writer,
            incoming_rx,
            cancel,
        } = self;
        let tx_only = PacketTransportTx { writer };
        (tx_only, ReceiverStream::new(incoming_rx), cancel)
    }
}

#[derive(Clone)]
pub struct VmDevice {
    transport: PacketTransportTx,
    thread_state: Weak<State>,
    cancel: Arc<tokio_util::sync::CancellationToken>, // Signals dispatcher to exit when VmDevice drops
    transport_cancel: Arc<tokio_util::sync::CancellationToken>, // Signals mux reader to exit when VmDevice drops
}

impl Drop for VmDevice {
    fn drop(&mut self) {
        eprintln!("!!! VmDevice DROP called !!!");
        // Only cancel if this is the last clone
        if Arc::strong_count(&self.cancel) == 1 {
            eprintln!("!!! VmDevice: Last clone dropping, canceling dispatcher !!!");
            self.cancel.cancel(); // Cancel dispatcher
            self.transport_cancel.cancel(); // Cancel mux reader
        } else {
            eprintln!(
                "!!! VmDevice: Clone dropped, {} remaining !!!",
                Arc::strong_count(&self.cancel) - 1
            );
        }
    }
}

/// Retry an asynchronous operation up to `limit` times.
async fn retry<F, G>(mut op: F, timeout: Duration, limit: usize) -> Option<G::Output>
where
    F: FnMut() -> G,
    G: std::future::Future,
{
    for _ in 0..limit {
        match tokio::time::timeout(timeout, op()).await {
            Ok(r) => return Some(r),
            Err(_) => (),
        }
    }
    None
}

impl VmDevice {
    pub fn from_transport(transport: PacketTransport) -> Self {
        let (tx_only, mut incoming, transport_cancel) = transport.split();

        let response_channels = std::array::from_fn(|_| ResponseChannel::None);
        let state = Arc::new(State {
            response_channels: Mutex::new(response_channels),
            streams_active: StreamsActive::default(),
        });
        let thread_state = Arc::downgrade(&state);
        let state_cloned = Arc::clone(&state);

        use rand::Rng;
        let dispatcher_id: u32 = rand::thread_rng().gen();
        debug!(
            "Created new VmDevice State [Dispatcher ID:{}]",
            dispatcher_id
        );
        let dispatcher_id_task = dispatcher_id;

        let cancel_token = tokio_util::sync::CancellationToken::new();
        let cancel_token_task = cancel_token.clone();
        tokio::spawn(async move {
            debug!("Dispatcher: [ID:{}] task started", dispatcher_id_task);
            use tokio_stream::StreamExt;
            loop {
                tokio::select! {
                    Some(reply) = incoming.next() => {
                        debug!("Dispatcher: [ID:{}] received packet id={}", dispatcher_id_task, reply.id);
                        debug!("Dispatcher: [ID:{}] packet type = {:?}", dispatcher_id_task, std::mem::discriminant(&reply.data));
                        let mut chans = state_cloned.response_channels.lock().unwrap();
                        let slot = &mut chans[usize::from(reply.id)];
                        debug!("Dispatcher: [ID:{}] slot state = {}", dispatcher_id_task, match slot {
                            ResponseChannel::None => "None",
                            ResponseChannel::Oneshot(_) => "Oneshot",
                            ResponseChannel::Stream(_) => "Stream",
                        });
                        let data = reply.data;

                        let err = match std::mem::take(slot) {
                            ResponseChannel::None => {
                                warn!("Packet received for unregistered slot {}", reply.id);
                                None
                            }
                            ResponseChannel::Oneshot(tx) => {
                                debug!("Dispatcher: [ID:{}] sending to oneshot channel", dispatcher_id_task);
                                let result = tx.send(data).err().map(|e| format!("{e:?}"));
                                debug!("Dispatcher: [ID:{}] oneshot send result = {:?}", dispatcher_id_task, if result.is_some() { "Error" } else { "Ok" });
                                result
                            }
                            ResponseChannel::Stream(tx) => {
                                let e = tx.try_send(data).err().map(|e| format!("{e:?}"));
                                *slot = ResponseChannel::Stream(tx);
                                e
                            }
                        };
                        if let Some(e) = err {
                            debug!("dispatcher failed to send reply: {e}");
                        } else {
                            debug!("Dispatcher: [ID:{}] successfully delivered packet id={}", dispatcher_id_task, reply.id);
                        }
                    }
                    _ = cancel_token_task.cancelled() => {
                        debug!("Dispatcher: [ID:{}] cancelled, exiting", dispatcher_id_task);
                        break;
                    }
                }
            }
            debug!("Dispatcher: [ID:{}] task exits", dispatcher_id_task);
        });

        VmDevice {
            transport: tx_only,
            thread_state,
            cancel: Arc::new(cancel_token),
            transport_cancel,
        }
    }

    pub async fn connect_usb(info: DeviceInfo) -> Result<Self> {
        let dev = info.open().await?;
        let iface = dev.claim_interface(0).await?;

        let in_ep = iface.endpoint::<Bulk, In>(0x81)?.reader(4096);
        let out_ep = iface.endpoint::<Bulk, Out>(0x01)?.writer(4096);

        let transport = PacketTransport::usb(in_ep, out_ep);
        let device = Self::from_transport(transport);
        let _ = device.clear_all_streams().await;
        Ok(device)
    }

    /// Create a VmDevice that communicates with a vision module through a mux/dongle
    pub async fn connect_via_mux(mux: MuxDevice, device_addr: [u8; 6]) -> Result<Self> {
        debug!("VmDevice::connect_via_mux() creating new device for {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            device_addr[0], device_addr[1], device_addr[2], device_addr[3], device_addr[4], device_addr[5]);
        let transport = PacketTransport::mux(mux, device_addr);
        let device = Self::from_transport(transport);

        // Send DisableAll command and wait for Ack
        // This ensures any lingering streams from a previous session are stopped
        // before we proceed with settings loading
        debug!("Sending DisableAll to clear any active streams...");
        if let None = retry(
            async || {
                device
                    .request(PacketData::StreamUpdate(StreamUpdate {
                        packet_id: PacketType::End(),
                        action: crate::packets::vm::StreamUpdateAction::DisableAll,
                    }))
                    .await
            },
            Duration::from_millis(2000),
            5,
        )
        .await
        {
            return Err(anyhow!("Failed to disable all streams"));
        }
        debug!("Ready to proceed with settings loading (slot reservation prevents conflicts)");

        Ok(device)
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

    fn get_oneshot_slot(&self) -> anyhow::Result<(ResponseSlot, oneshot::Receiver<PacketData>)> {
        if let Some(thread_state) = self.thread_state.upgrade() {
            let mut response_channels = thread_state.response_channels.lock().unwrap();
            // Slot reservation: IDs 0-207 for control, 208-255 for streaming
            // Control requests (oneshot) use the low range
            const CONTROL_ID_MAX: usize = 207;

            let available = response_channels
                .iter()
                .take(CONTROL_ID_MAX + 1)
                .filter(|c| matches!(c, ResponseChannel::None))
                .count();
            debug!("get_oneshot_slot: {} slots available", available);
            for (i, c) in response_channels
                .iter_mut()
                .enumerate()
                .take(CONTROL_ID_MAX + 1)
            {
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
        } else {
            debug!("ERROR: thread_state upgrade failed - State was dropped!");
        }
        Err(anyhow::anyhow!("Failed to allocate request id"))
    }

    fn get_stream_slot(
        &self,
        buffer: usize,
    ) -> Result<(ResponseSlot, mpsc::Receiver<PacketData>), anyhow::Error> {
        if let Some(thread_state) = self.thread_state.upgrade() {
            let mut response_channels = thread_state.response_channels.lock().unwrap();
            // Slot reservation: IDs 208-255 for streaming (48 slots), 0-207 for control
            // Streaming requests use the high range
            const STREAMING_ID_MIN: usize = 208;
            for (i, c) in response_channels
                .iter_mut()
                .enumerate()
                .skip(STREAMING_ID_MIN)
            {
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

    pub async fn read_config(&self, kind: crate::packets::vm::ConfigKind) -> Result<GeneralConfig> {
        let r = self
            .request(PacketData::ReadConfig(kind))
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

    pub async fn read_prop(&self, kind: crate::packets::vm::PropKind) -> Result<Props> {
        let r = self
            .request(PacketData::ReadProp(kind))
            .await?
            .read_prop_response()
            .with_context(|| "unexpected response")?;
        info!("prop: {:?}", r);
        Ok(r)
    }

    /// Read all configuration values and return them as a single struct
    pub async fn read_all_config(&self) -> Result<GeneralSettings> {
        let impact_threshold = self.read_config(ConfigKind::ImpactThreshold).await?;
        let suppress_ms = self.read_config(ConfigKind::SuppressMs).await?;
        let accel_config = self.read_config(ConfigKind::AccelConfig).await?;
        let gyro_config = self.read_config(ConfigKind::GyroConfig).await?;
        let camera_model_nf = self.read_config(ConfigKind::CameraModelNf).await?;
        let camera_model_wf = self.read_config(ConfigKind::CameraModelWf).await?;
        let stereo_iso = self.read_config(ConfigKind::StereoIso).await?;

        Ok(GeneralSettings {
            impact_threshold: match impact_threshold {
                GeneralConfig::ImpactThreshold(val) => val,
                _ => anyhow::bail!("Unexpected config variant for ImpactThreshold"),
            },
            suppress_ms: match suppress_ms {
                GeneralConfig::SuppressMs(val) => val,
                _ => anyhow::bail!("Unexpected config variant for SuppressMs"),
            },
            accel_config: match accel_config {
                GeneralConfig::AccelConfig(val) => val,
                _ => anyhow::bail!("Unexpected config variant for AccelConfig"),
            },
            gyro_config: match gyro_config {
                GeneralConfig::GyroConfig(val) => val,
                _ => anyhow::bail!("Unexpected config variant for GyroConfig"),
            },
            camera_model_nf: match camera_model_nf {
                GeneralConfig::CameraModelNf(val) => val,
                _ => anyhow::bail!("Unexpected config variant for CameraModelNf"),
            },
            camera_model_wf: match camera_model_wf {
                GeneralConfig::CameraModelWf(val) => val,
                _ => anyhow::bail!("Unexpected config variant for CameraModelWf"),
            },
            stereo_iso: match stereo_iso {
                GeneralConfig::StereoIso(val) => val,
                _ => anyhow::bail!("Unexpected config variant for StereoIso"),
            },
        })
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
            info!(
                "Requesting stream: type={:?}, slot_id={}",
                stream_type, slot.id
            );
            self.transport
                .writer
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
        self.transport
            .writer
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

    pub async fn clear_all_streams(&self) -> Result<()> {
        // Mark all streams as inactive to stop new packets
        if let Some(thread_state) = self.thread_state.upgrade() {
            for stream_type in 0u8..=4u8 {
                let packet_type = match stream_type {
                    0 => PacketType::ObjectReport(),
                    1 => PacketType::CombinedMarkersReport(),
                    2 => PacketType::PocMarkersReport(),
                    3 => PacketType::AccelReport(),
                    4 => PacketType::ImpactReport(),
                    _ => continue,
                };

                if thread_state.streams_active[packet_type].load(Ordering::Relaxed) {
                    info!("Clearing active stream: {:?}", packet_type);
                    thread_state.streams_active[packet_type].store(false, Ordering::Relaxed);
                }
            }
        }

        if let None = retry(
            async || {
                self.request(PacketData::StreamUpdate(StreamUpdate {
                    packet_id: PacketType::End(),
                    action: crate::packets::vm::StreamUpdateAction::DisableAll,
                }))
                .await
            },
            Duration::from_millis(2000),
            5,
        )
        .await
        {
            return Err(anyhow!("Failed to disable all streams"));
        }
        Ok(())
    }
}

pub struct PacketStream {
    slot: ResponseSlot,
    stream_type: PacketType,
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
            // Send Disable packet for this specific stream
            let sender = self.sender.clone();
            let stream_type = self.stream_type;
            tokio::spawn(async move {
                match retry(
                    async || {
                        sender
                            .send(Packet {
                                id: 255,
                                data: PacketData::StreamUpdate(StreamUpdate {
                                    packet_id: stream_type,
                                    action: crate::packets::vm::StreamUpdateAction::Disable,
                                }),
                            })
                            .await
                    },
                    Duration::from_millis(2000),
                    5,
                )
                .await
                {
                    Some(Ok(())) => {}
                    Some(Err(e)) => {
                        warn!("Failed to send Disable packet for {:?}, {}", stream_type, e);
                    }
                    None => {
                        warn!("Failed to send Disable packet for {:?}", stream_type);
                    }
                }
            });
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

// mux Device for dongle-fw

use crate::packets::mux::{MuxMsg, SendTo, MAX_DEVICES};
use heapless::Vec as HVec;

pub struct MuxTransport {
    writer: mpsc::Sender<MuxMsg>,
    incoming_rx: mpsc::Receiver<MuxMsg>,
}

impl MuxTransport {
    pub fn usb(mut in_ep: EndpointRead<Bulk>, mut out_ep: EndpointWrite<Bulk>) -> Self {
        let (writer, mut writer_rx) = mpsc::channel::<MuxMsg>(64);
        let (incoming_tx, incoming_rx) = mpsc::channel::<MuxMsg>(128);

        // Writer task
        tokio::spawn(async move {
            let mut raw = Vec::with_capacity(1024);
            while let Some(msg) = writer_rx.recv().await {
                raw.clear();
                if let Err(e) = postcard::to_io(&msg, &mut raw) {
                    error!("mux postcard serialize failed: {e}");
                    continue;
                }
                if let Err(e) = out_ep.write_all(&raw).await {
                    error!("mux usb write_all failed: {e}");
                    continue;
                }
                if let Err(e) = out_ep.flush_end_async().await {
                    error!("mux usb flush_end failed: {e:?}");
                    continue;
                }
            }
            info!("mux usb writer exits");
        });

        // Reader task
        tokio::spawn(async move {
            let mut reader = in_ep.until_short_packet();
            let mut io_errs = 0u8;
            loop {
                let mut buf = Vec::with_capacity(256);
                match reader.read_to_end(&mut buf).await {
                    Err(e) => {
                        use std::io::ErrorKind::*;
                        match e.kind() {
                            TimedOut | WouldBlock => {
                                io_errs = 0;
                                continue;
                            }
                            BrokenPipe | UnexpectedEof | ConnectionReset => break,
                            _ => {
                                if io_errs < 3 {
                                    warn!("mux usb read error (ignored): {e}");
                                    io_errs += 1;
                                    continue;
                                }
                                error!("mux usb read error: {e}");
                                break;
                            }
                        }
                    }
                    Ok(_) => {
                        io_errs = 0;
                    }
                }
                if let Err(e) = reader.consume_end() {
                    warn!("mux usb consume_end failed: {e:?}");
                    continue;
                }
                match postcard::from_bytes::<MuxMsg>(&buf) {
                    Ok(msg) => {
                        if incoming_tx.send(msg).await.is_err() {
                            break;
                        }
                    }
                    Err(e) => error!("mux postcard decode failed: {e:?}"),
                }
            }
            info!("mux usb reader exits");
        });

        MuxTransport {
            writer,
            incoming_rx,
        }
    }

    pub fn split(self) -> (mpsc::Sender<MuxMsg>, ReceiverStream<MuxMsg>) {
        let MuxTransport {
            writer,
            incoming_rx,
        } = self;
        (writer, ReceiverStream::new(incoming_rx))
    }
}

#[derive(Clone, Debug)]
pub struct MuxDevice {
    writer: mpsc::Sender<MuxMsg>,
    response_rx: Arc<tokio::sync::Mutex<ReceiverStream<MuxMsg>>>,
    dev: Device,
    ctrl_if: Interface,
}

impl MuxDevice {
    async fn ctrl_recv_polling(&self, timeout: std::time::Duration) -> Result<UsbMuxCtrlMsg> {
        let idx = self.ctrl_if.interface_number() as u16;
        let start = std::time::Instant::now();
        loop {
            let elapsed = start.elapsed();
            if elapsed >= timeout {
                return Err(anyhow!("timeout waiting for ctrl event"));
            }
            let slice = (timeout - elapsed).min(std::time::Duration::from_millis(500));
            match self
                .ctrl_if
                .control_in(
                    ControlIn {
                        control_type: ControlType::Vendor,
                        recipient: Recipient::Interface,
                        request: Self::USB_MUX_CTRL_REQ_RECV,
                        value: 0,
                        index: idx,
                        length: 256,
                    },
                    slice,
                )
                .await
            {
                Ok(data) => {
                    if data.is_empty() {
                        continue;
                    }
                    break postcard::from_bytes::<UsbMuxCtrlMsg>(&data)
                        .map_err(|e| anyhow!("ctrl decode failed: {e}"));
                }
                Err(_) => {
                    continue;
                }
            }
        }
    }
    // USB Mux control over EP0 (Interface recipient)
    const USB_MUX_CTRL_REQ_SEND: u8 = 0x30; // vendor request to send ctrl msg
    const USB_MUX_CTRL_REQ_RECV: u8 = 0x31; // vendor request to receive ctrl msg

    async fn ctrl_send(&self, msg: UsbMuxCtrlMsg) -> Result<()> {
        use std::time::Duration;
        let data = postcard::to_allocvec(&msg).map_err(|e| anyhow!("ctrl encode failed: {e}"))?;
        self.ctrl_if
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Interface,
                    request: Self::USB_MUX_CTRL_REQ_SEND,
                    value: 0,
                    index: self.ctrl_if.interface_number() as u16,
                    data: &data,
                },
                Duration::from_millis(1000),
            )
            .await
            .map_err(|e| anyhow!("control_out failed: {e}"))?;
        Ok(())
    }

    async fn ctrl_recv(&self, timeout: std::time::Duration) -> Result<UsbMuxCtrlMsg> {
        let data = self
            .ctrl_if
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Interface,
                    request: Self::USB_MUX_CTRL_REQ_RECV,
                    value: 0,
                    index: self.ctrl_if.interface_number() as u16,
                    length: 256,
                },
                timeout,
            )
            .await
            .map_err(|e| anyhow!("control_in failed: {e}"))?;
        postcard::from_bytes::<UsbMuxCtrlMsg>(&data).map_err(|e| anyhow!("ctrl decode failed: {e}"))
    }
    pub fn from_transport(transport: MuxTransport, dev: Device, ctrl_if: Interface) -> Self {
        let (writer, incoming) = transport.split();

        MuxDevice {
            writer,
            response_rx: Arc::new(tokio::sync::Mutex::new(incoming)),
            dev,
            ctrl_if,
        }
    }

    pub async fn connect_usb(info: DeviceInfo) -> Result<Self> {
        let dev = info.open().await?;

        // Probe interfaces to find the one that has our bulk endpoints (0x01 OUT, 0x81 IN).
        for idx in 0u8..8 {
            let iface = match dev.claim_interface(idx).await {
                Ok(i) => i,
                Err(_) => continue,
            };

            let in_ep = match iface.endpoint::<Bulk, In>(0x81) {
                Ok(e) => e.reader(4096),
                Err(_) => {
                    drop(iface);
                    continue;
                }
            };
            let out_ep = match iface.endpoint::<Bulk, Out>(0x01) {
                Ok(e) => e.writer(4096),
                Err(_) => {
                    drop(iface);
                    continue;
                }
            };

            let ctrl_if = iface.clone();
            let transport = MuxTransport::usb(in_ep, out_ep);
            return Ok(Self::from_transport(transport, dev, ctrl_if));
        }
        Err(anyhow!(
            "failed to find mux interface with required endpoints"
        ))
    }

    async fn send_msg(&self, msg: MuxMsg) -> Result<()> {
        self.writer
            .send(msg)
            .await
            .map_err(|e| anyhow!("send failed: {e}"))?;
        Ok(())
    }

    pub async fn receive_msg(&self) -> Result<MuxMsg> {
        use tokio_stream::StreamExt;
        let mut rx = self.response_rx.lock().await;
        let msg = rx.next().await.ok_or_else(|| anyhow!("channel closed"))?;
        debug!("MuxDevice::receive_msg got message");
        Ok(msg)
    }

    pub async fn request_devices(&self) -> Result<HVec<[u8; 6], MAX_DEVICES>> {
        self.send_msg(MuxMsg::RequestDevices).await?;

        let total_timeout = tokio::time::Duration::from_secs(1);

        let receive_until_snapshot = async {
            loop {
                match self.receive_msg().await {
                    Ok(MuxMsg::DevicesSnapshot(devices)) => return Ok(devices),
                    Ok(_other) => continue,
                    Err(e) => return Err(anyhow!("receive_msg error: {e}")),
                }
            }
        };

        match tokio::time::timeout(total_timeout, receive_until_snapshot).await {
            Ok(Ok(devices)) => Ok(devices),
            Ok(Err(e)) => Err(e),
            Err(_) => Err(anyhow!("timeout waiting for DevicesSnapshot")),
        }
    }

    pub async fn send_to(&self, dev: [u8; 6], pkt: Packet) -> Result<()> {
        let msg = MuxMsg::SendTo(SendTo { dev, pkt });
        self.send_msg(msg).await
    }

    pub async fn read_version(&self) -> Result<crate::packets::mux::Version> {
        self.send_msg(MuxMsg::ReadVersion()).await?;

        match self.receive_msg().await? {
            MuxMsg::ReadVersionResponse(version) => Ok(version),
            other => Err(anyhow!("unexpected response: {other:?}")),
        }
    }

    /// Start pairing mode on the dongle with a timeout (ms).
    pub async fn start_pairing(&self, timeout_ms: u32) -> Result<()> {
        use protodongers::control::usb_mux::StartPairing;
        self.ctrl_send(UsbMuxCtrlMsg::StartPairing(StartPairing { timeout_ms }))
            .await?;
        match self
            .ctrl_recv_polling(std::time::Duration::from_secs(2))
            .await?
        {
            UsbMuxCtrlMsg::StartPairingResponse => Ok(()),
            other => Err(anyhow!("unexpected ctrl response: {other:?}")),
        }
    }

    /// Cancel pairing mode on the dongle.
    pub async fn cancel_pairing(&self) -> Result<()> {
        self.ctrl_send(UsbMuxCtrlMsg::CancelPairing).await
    }

    /// Wait for a pairing event from the dongle.
    pub async fn wait_pairing_event(&self) -> Result<PairingEvent> {
        loop {
            match self
                .ctrl_recv_polling(std::time::Duration::from_secs(120))
                .await?
            {
                UsbMuxCtrlMsg::PairingResult(Ok(addr)) => return Ok(PairingEvent::Result(addr)),
                UsbMuxCtrlMsg::PairingResult(Err(PairingError::Timeout)) => {
                    return Ok(PairingEvent::Timeout)
                }
                UsbMuxCtrlMsg::PairingResult(Err(PairingError::Cancelled)) => {
                    return Ok(PairingEvent::Cancelled)
                }
                _ => { /* ignore unrelated ctrl msgs */ }
            }
        }
    }

    pub async fn clear_bonds(&self) -> Result<()> {
        self.ctrl_send(UsbMuxCtrlMsg::ClearBonds).await?;
        match self
            .ctrl_recv_polling(std::time::Duration::from_secs(2))
            .await?
        {
            UsbMuxCtrlMsg::ClearBondsResponse(Ok(())) => Ok(()),
            UsbMuxCtrlMsg::ClearBondsResponse(Err(ClearBondsError::Failed)) => {
                Err(anyhow!("Failed to clear bonds"))
            }
            other => Err(anyhow!("unexpected ctrl response: {other:?}")),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PairingEvent {
    Result([u8; 6]),
    Timeout,
    Cancelled,
}

/// Represents either a direct USB connection or a mux-mediated connection to a vision module
#[derive(Clone, Debug)]
pub enum VmConnectionInfo {
    DirectUsb(DeviceInfo),
    ViaMux {
        mux: MuxDevice,
        device_addr: [u8; 6],
    },
}

impl VmConnectionInfo {
    /// Connect to the vision module using the appropriate method
    pub async fn connect(self) -> Result<VmDevice> {
        match self {
            VmConnectionInfo::DirectUsb(info) => VmDevice::connect_usb(info).await,
            VmConnectionInfo::ViaMux { mux, device_addr } => {
                VmDevice::connect_via_mux(mux, device_addr).await
            }
        }
    }

    /// Get a display string for this device
    pub fn display_string(&self) -> String {
        match self {
            VmConnectionInfo::DirectUsb(info) => {
                format!(
                    "Vision Module {:04X}:{:04X}",
                    info.vendor_id(),
                    info.product_id()
                )
            }
            VmConnectionInfo::ViaMux { device_addr, .. } => {
                format!(
                    "VM via Mux {:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
                    device_addr[0],
                    device_addr[1],
                    device_addr[2],
                    device_addr[3],
                    device_addr[4],
                    device_addr[5]
                )
            }
        }
    }
}
