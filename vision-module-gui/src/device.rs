use std::{borrow::Cow, time::Duration};

use anyhow::{Context, Result};
use serialport::ClearBuffer::Input;
use tokio::sync::{mpsc, oneshot};

use crate::packet::{MotData, ObjectReportRequest, Packet, Port, Register};

type Message = (Packet, oneshot::Sender<Result<Packet>>);

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
                if sent.is_err() {
                    eprintln!("device thread failed to send reply");
                }
            }
            eprintln!("device thread for {:?} exiting", port.name());
        });
        Ok(Self(sender))
    }

    pub async fn request(&self, packet: Packet) -> Result<Packet> {
        let oneshot = oneshot::channel();
        self.0.send((packet, oneshot.0)).await?;
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

    pub async fn get_frame(&self) -> Result<(MotData, MotData)> {
        let r = self
            .request(Packet::ObjectReportRequest(ObjectReportRequest{}))
            .await?
            .object_report()
            .with_context(|| "unexpected response")?;
        Ok((r.mot_data_nf, r.mot_data_wf))
    }

    pub async fn product_id(&self, port: Port) -> Result<u16> {
        let a = self.read_register(port, 0x00, 0x02).await?;
        let b = self.read_register(port, 0x00, 0x03).await?;
        Ok(u16::from_le_bytes([a, b]))
    }
}
