use std::{borrow::Cow, io::{BufReader, ErrorKind, Read, Write}, net::TcpStream, pin::Pin, sync::{atomic::{AtomicBool, Ordering}, Arc, Mutex}, task::Poll, time::Duration};
use anyhow::{anyhow, Context, Result};
use pin_project::{pin_project, pinned_drop};
use serial2;
use tokio::sync::{mpsc, oneshot};
use tokio_stream::{wrappers::ReceiverStream, Stream, StreamExt};
use tracing::{debug, error, info, trace, warn};

use crate::packet::{AccelReport, CombinedMarkersReport, GeneralConfig, MotData, ObjectReport, ObjectReportRequest, Packet, PacketData, Port, Register, StreamUpdate, WriteRegister};

#[derive(Default)]
enum ResponseChannel {
    #[default]
    None,
    Oneshot(oneshot::Sender<PacketData>),
    Stream(mpsc::Sender<PacketData>),
}

#[derive(Clone)]
pub struct UsbDevice {
    to_thread: mpsc::Sender<Packet>,
    thread_state: Arc<State>,
}

struct State {
    // id 255 is reserved for requests that don't care for a response
    response_channels: Mutex<[ResponseChannel; 255]>,
    mot_data_stream: AtomicBool,
    impact_stream: AtomicBool,
    combined_markers_stream: AtomicBool,
    accel_stream: AtomicBool,
}

/// A helper struct to deal with cancellation
struct ResponseSlot {
    thread_state: Arc<State>,
    id: u8,
    finished: bool,
}

impl Drop for ResponseSlot {
    fn drop(&mut self) {
        // If the future holding the slot was cancelled, remove the sender from the
        // channel.
        if !self.finished {
            self.thread_state.response_channels.lock().unwrap()[usize::from(self.id)] = ResponseChannel::None;
        }
    }
}

impl UsbDevice {
    /// Connect to the device using the serial port at `path`. Starts two background threads to
    /// service reads and writes.
    pub async fn connect_serial<'a>(path: impl Into<Cow<'a, str>>) -> Result<Self> {
        let path = path.into();
        info!("Connecting to {path}...");
        let mut read_port = serial2::SerialPort::open(path.as_ref(), |mut settings: serial2::Settings| {
            settings.set_raw();
            settings.set_baud_rate(115200)?;
            settings.set_char_size(serial2::CharSize::Bits7);
            settings.set_stop_bits(serial2::StopBits::Two);
            settings.set_parity(serial2::Parity::Odd);
            settings.set_flow_control(serial2::FlowControl::RtsCts);
            Ok(settings)
        })?;

        read_port.set_read_timeout(Duration::from_millis(10))?;
        // why is this here
        // read_port.set_write_timeout(Duration::from_millis(0))?;

        let mut port = tokio::task::spawn_blocking(move || -> Result<_> {
            let mut buf = vec![0xff];
            Packet { id: 0, data: PacketData::StreamUpdate(StreamUpdate { mask: 0xff, active: false }) }.serialize(&mut buf);
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
        }).await??;

        port.set_read_timeout(Duration::from_millis(3000))?;
        port.set_dtr(true).unwrap();
        let writer = port.try_clone().context("Failed to clone serial port")?;
        let reader = port;
        Ok(Self::new(reader, writer))
    }

    pub fn connect_tcp(addr: &str) -> Result<Self> {
        info!("Connecting to {addr}...");
        let conn = TcpStream::connect(addr)?;
        conn.set_read_timeout(Some(Duration::from_millis(3000)))?;
        let conn2 = conn.try_clone().context("Failed to clone tcp stream")?;
        Ok(Self::new(conn, conn2))
    }

    pub fn new<R, W>(reader: R, writer: W) -> Self
    where
        R: Read + Send + 'static,
        W: Write + Send + 'static,
    {
        let response_channels = std::array::from_fn(|_| ResponseChannel::None);
        let state = Arc::new(State {
            response_channels: Mutex::new(response_channels),
            mot_data_stream: AtomicBool::new(false),
            impact_stream: AtomicBool::new(false),
            combined_markers_stream: AtomicBool::new(false),
            accel_stream: AtomicBool::new(false),
        });
        let thread_state = Arc::clone(&state);

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

                debug!("write id={} len={}", pkt.id, buf.len());
                let r = writer.write_all(&buf).and_then(|_| writer.flush());
                if let Err(e) = r {
                    error!("device thread failed to write: {e}");
                }
            }
            info!("device writer thread exiting");
        });

        // Reader thread.
        std::thread::spawn(move || {
            let mut reader = BufReader::new(reader);
            let mut buf = vec![];
            let mut io_error_count = 0;
            loop {
                if Arc::strong_count(&state) == 1 {
                    // We are the only ones with a reference to the state
                    break;
                }
                let reply: Result<_, std::io::Error> = (|| {
                    buf.clear();
                    buf.resize(2, 0);
                    reader.read_exact(&mut buf)?;
                    let len = u16::from_le_bytes([buf[0], buf[1]]);
                    if len == 0 {
                        return Err(std::io::Error::new(ErrorKind::InvalidData, anyhow!("packet length is 0")));
                    }
                    trace!("read len={}", len*2);
                    buf.resize(usize::from(len * 2), 0);
                    reader.read_exact(&mut buf[2..])?;
                    Ok(Packet::parse(&mut &buf[..])
                        .with_context(|| format!("failed to parse packet: {:?}", buf.clone()))
                        .inspect(|p| trace!("read id={} type={:?}", p.id, p.ty())))
                })();
                let reply = match reply {
                    // IO error
                    Err(e) => {
                        if e.kind() == ErrorKind::TimedOut || e.kind() == ErrorKind::WouldBlock {
                            debug!("read timed out");
                            io_error_count = 0;
                            continue;
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
                        error!("{}", e);
                        continue;
                    }
                    Ok(r) => r,
                };
                let mut response_channels = state.response_channels.lock().unwrap();
                let response_sender = &mut response_channels[usize::from(reply.id)];
                let e = match std::mem::take(response_sender) {
                    ResponseChannel::None => None,
                    ResponseChannel::Oneshot(oneshot) => oneshot.send(reply.data).err().map(|e| format!("{e:?}")),
                    ResponseChannel::Stream(sender) => {
                        let e = sender.try_send(reply.data).err().map(|e| format!("{e:?}"));
                        *response_sender = ResponseChannel::Stream(sender);
                        e
                    }
                };
                if let Some(e) = e {
                    info!("device reader thread failed to send reply: {e:?}");
                }
            }
            info!("device reader thread exiting");
        });
        Self {
            to_thread: sender,
            thread_state,
        }
    }

    fn get_oneshot_slot(&self) -> (ResponseSlot, oneshot::Receiver<PacketData>) {
        let mut response_channels = self.thread_state.response_channels.lock().unwrap();
        for (i, c) in response_channels.iter_mut().enumerate() {
            if let ResponseChannel::None = c {
                let (send, receiver) = oneshot::channel();
                *c = ResponseChannel::Oneshot(send);
                return (
                    ResponseSlot {
                        thread_state: self.thread_state.clone(),
                        id: i as u8,
                        finished: false,
                    },
                    receiver,
                );
            }
        }
        panic!("Failed to allocate request id");
    }

    fn get_stream_slot(&self, buffer: usize) -> (ResponseSlot, mpsc::Receiver<PacketData>) {
        let mut response_channels = self.thread_state.response_channels.lock().unwrap();
        for (i, c) in response_channels.iter_mut().enumerate() {
            if let ResponseChannel::None = c {
                let (send, receiver) = mpsc::channel(buffer);
                *c = ResponseChannel::Stream(send);
                return (
                    ResponseSlot {
                        thread_state: self.thread_state.clone(),
                        id: i as u8,
                        finished: false,
                    },
                    receiver,
                );
            }
        }
        panic!("Failed to allocate request id");
    }

    pub async fn request(&self, packet: PacketData) -> Result<PacketData> {
        let (mut response_slot, receiver) = self.get_oneshot_slot();
        self.to_thread.send(Packet { id: response_slot.id, data: packet }).await?;
        let result = receiver.await;
        response_slot.finished = true;
        Ok(result?)
    }

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
        let pkt = Packet {
            id: 255,
            data,
        };
        self.to_thread.send(pkt).await?;
        Ok(())
    }

    pub async fn read_config(&self) -> Result<GeneralConfig> {
        let r = self
            .request(PacketData::ReadConfig)
            .await?
            .read_config_response()
            .with_context(|| "unexpected response")?;
        info!("config: {:?}", r);
        Ok(r)
    }

    pub async fn write_config(&self, config: GeneralConfig) -> Result<()> {
        let data = PacketData::WriteConfig(config);
        let pkt = Packet {
            id: 255,
            data,
        };
        self.to_thread.send(pkt).await?;
        Ok(())
    }

    pub async fn get_frame(&self) -> Result<([MotData; 16], [MotData; 16])> {
        let r = self
            .request(PacketData::ObjectReportRequest(ObjectReportRequest {}))
            .await?
            .object_report()
            .with_context(|| "unexpected response")?;
        Ok((r.mot_data_nf, r.mot_data_wf))
    }

    pub async fn stream_mot_data(&self) -> Result<impl Stream<Item = ObjectReport> + Send + Sync> {
        if self.thread_state.mot_data_stream.swap(true, Ordering::Relaxed) {
            return Err(anyhow!("cannot have more than one mot data stream"));
        }
        let (slot, receiver) = self.get_stream_slot(2);
        self.to_thread.send(Packet { id: slot.id, data: PacketData::StreamUpdate(StreamUpdate { mask: 0b0001, active: true }) }).await?;
        Ok(PacketStream { slot, receiver: ReceiverStream::new(receiver), stream_type: 0 }.map(|x| {
            x.object_report().unwrap()
        }))
    }

    pub async fn stream_combined_markers(&self) -> Result<impl Stream<Item = CombinedMarkersReport> + Send + Sync> {
        if self.thread_state.combined_markers_stream.swap(true, Ordering::Relaxed) {
            return Err(anyhow!("cannot have more than one aim stream"));
        }
        let (slot, receiver) = self.get_stream_slot(2);
        self.to_thread.send(Packet { id: slot.id, data: PacketData::StreamUpdate(StreamUpdate { mask: 0b0010, active: true }) }).await?;
        Ok(PacketStream { slot, receiver: ReceiverStream::new(receiver), stream_type: 1 }.map(|x| {
            x.combined_markers_report().unwrap()
        }))
    }

    pub async fn stream_accel(&self) -> Result<impl Stream<Item = AccelReport> + Send + Sync> {
        if self.thread_state.accel_stream.swap(true, Ordering::Relaxed) {
            return Err(anyhow!("cannot have more than one accel stream"));
        }
        let (slot, receiver) = self.get_stream_slot(2);
        self.to_thread.send(Packet { id: slot.id, data: PacketData::StreamUpdate(StreamUpdate { mask: 0b0100, active: true }) }).await?;
        Ok(PacketStream { slot, receiver: ReceiverStream::new(receiver), stream_type: 2 }.map(|x| {
            x.accel_report().unwrap()
        }))
    }

    pub async fn stream_impact(&self) -> Result<impl Stream<Item = ()> + Send + Sync> {
        if self.thread_state.impact_stream.swap(true, Ordering::Relaxed) {
            return Err(anyhow!("cannot have more than one impact stream"));
        }
        let (slot, receiver) = self.get_stream_slot(2);
        self.to_thread.send(Packet { id: slot.id, data: PacketData::StreamUpdate(StreamUpdate { mask: 0b1000, active: true }) }).await?;
        Ok(PacketStream { slot, receiver: ReceiverStream::new(receiver), stream_type: 3 }.map(|_| {
            ()
        }))
    }

    pub async fn flash_settings(&self) -> Result<()> {
        self.to_thread.send(Packet {
            id: 255,
            data: PacketData::FlashSettings,
        }).await?;
        Ok(())
    }
}

#[pin_project(PinnedDrop)]
pub struct PacketStream {
    stream_type: u8,
    slot: ResponseSlot,
    #[pin]
    receiver: ReceiverStream<PacketData>,
}

impl Stream for PacketStream {
    type Item = PacketData;

    fn poll_next(self: Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> Poll<Option<Self::Item>> {
        self.project().receiver.poll_next(cx)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.receiver.size_hint()
    }
}

#[pinned_drop]
impl PinnedDrop for PacketStream {
    fn drop(self: Pin<&mut Self>) {
        if self.stream_type == 0 {
            self.slot.thread_state.mot_data_stream.store(false, Ordering::Relaxed);
        } else if self.stream_type == 1 {
            self.slot.thread_state.combined_markers_stream.store(false, Ordering::Relaxed);
        } else if self.stream_type == 2 {
            self.slot.thread_state.accel_stream.store(false, Ordering::Relaxed);
        } else if self.stream_type == 3 {
            self.slot.thread_state.impact_stream.store(false, Ordering::Relaxed);
        }
    }
}

macro_rules! read_register_spec {
    ($name:ident : $ty:ty = $bank:literal; [$($addr:literal),*]) => {
        pub(crate) async fn $name(&self, port: Port) -> ::anyhow::Result<$ty> {
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
        pub(crate) async fn $name(&self, port: Port, value: $ty) -> ::anyhow::Result<()> {
            let bytes = <$ty>::to_le_bytes(value);
            for (byte, addr) in ::std::iter::zip(bytes, [$($addr),*]) {
                self.write_register(port, $bank, addr, byte).await?;
            }
            Ok(())
        }
    }
}

impl UsbDevice {
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
}
