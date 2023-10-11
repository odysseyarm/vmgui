use std::{borrow::Cow, time::Duration};

use anyhow::{Context, Result};
use serialport::ClearBuffer::Input;
use tokio::sync::{mpsc, oneshot};

use crate::packet::{MotData, ObjectReportRequest, Packet, Port, Register, WriteRegister};

type Message = (Packet, Option<oneshot::Sender<Result<Packet>>>);

#[derive(Clone)]
pub struct UsbDevice(mpsc::Sender<Message>);

impl UsbDevice {
    pub fn connect<'a>(path: impl Into<Cow<'a, str>>) -> Result<Self> {
        let path = path.into();
        eprintln!("Connecting to {path}...");
        let mut port = serialport::new(path, 115200)
            .timeout(Duration::from_secs(3))
            .open()?;

        port.clear(Input)?;

        port.write_data_terminal_ready(true)?;

        let (sender, mut receiver) = mpsc::channel::<Message>(16);
        std::thread::spawn(move || {
            let mut buf = vec![];
            loop {
                let Some((pkt, reply_sender)) = receiver.blocking_recv() else {
                    break;
                };

                buf.clear();
                buf.push(0xff);
                pkt.serialize(&mut buf);

                let Some(reply_sender) = reply_sender else {
                    let r = port.write_all(&buf).and_then(|_| port.flush());
                    if let Err(e) = r {
                        eprintln!("device thread failed to write: {e}");
                    }
                    continue;
                };

                let reply = (|| {
                    port.write_all(&buf)?;
                    port.flush()?;

                    buf.clear();
                    buf.resize(2, 0);
                    port.read_exact(&mut buf)?;
                    let len = u16::from_le_bytes([buf[0], buf[1]]);
                    buf.resize(usize::from(len * 2), 0);
                    port.read_exact(&mut buf[2..])?;
                    let resp =
                        Packet::parse(&mut &buf[..]).with_context(|| "failed to parse packet")?;
                    Ok(resp)
                })();
                let sent = reply_sender.send(reply);
                if let Err(e) = sent {
                    eprintln!("device thread failed to send reply: {e:?}");
                }
            }
            eprintln!("device thread for {:?} exiting", port.name());
        });
        Ok(Self(sender))
    }

    pub async fn request(&self, packet: Packet) -> Result<Packet> {
        let oneshot = oneshot::channel();
        self.0.send((packet, Some(oneshot.0))).await?;
        oneshot.1.await?
    }

    pub async fn read_register(&self, port: Port, bank: u8, address: u8) -> Result<u8> {
        let r = self
            .request(Packet::ReadRegister(Register {
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
        let pkt = Packet::WriteRegister(WriteRegister {
            port,
            bank,
            address,
            data,
        });
        self.0.send((pkt, None)).await?;
        Ok(())
    }

    pub async fn get_frame(&self) -> Result<([MotData; 16], [MotData; 16])> {
        let r = self
            .request(Packet::ObjectReportRequest(ObjectReportRequest {}))
            .await?
            .object_report()
            .with_context(|| "unexpected response")?;
        Ok((r.mot_data_nf, r.mot_data_wf))
    }
}

macro_rules! read_register_spec {
    ($name:ident : $ty:ty = $bank:literal; [$($addr:literal),*]) => {
        pub async fn $name(&self, port: Port) -> ::anyhow::Result<$ty> {
            Ok(<$ty>::from_le_bytes([
                $( self.read_register(port, $bank, $addr).await? ),*
            ]))
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
    read_register_spec!(product_id: u16 = 0x00; [0x02, 0x03]);
    read_register_spec!(resolution_x: u16 = 0x0c; [0x60, 0x61]);
    read_register_spec!(resolution_y: u16 = 0x0c; [0x62, 0x63]);
    write_register_spec!(set_resolution_x: u16 = 0x0c; [0x60, 0x61]);
    write_register_spec!(set_resolution_y: u16 = 0x0c; [0x62, 0x63]);
    read_register_spec!(gain_1: u8 = 0x01; [0x05]);
    read_register_spec!(gain_2: u8 = 0x01; [0x06]);
    write_register_spec!(set_gain_1: u8 = 0x0c; [0x0b]);
    write_register_spec!(set_gain_2: u8 = 0x0c; [0x0c]);
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
}
