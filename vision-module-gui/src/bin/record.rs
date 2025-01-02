use std::{process::ExitCode, time::UNIX_EPOCH};

use ats_usb::device::UsbDevice;
use serialport::SerialPortType;
use tokio::sync::mpsc::{self, Sender};
use tokio_stream::StreamExt;
#[allow(unused_imports)]
use tracing::Level;
#[allow(unused_imports)]
use tracing_subscriber::EnvFilter;

fn get_ports() -> Vec<serialport::SerialPortInfo> {
    let ports = serialport::available_ports().expect("Failed to list serial ports");
    let ports = ports
        .into_iter()
        .filter(|port| {
            match &port.port_type {
                SerialPortType::UsbPort(port_info) => {
                    if port_info.vid == 0x1915 && port_info.pid == 0x520F || port_info.pid == 0x5210
                    {
                        if let Some(i) = port_info.interface {
                            // interface 0: cdc acm module
                            // interface 1: cdc acm module functional subordinate interface
                            // interface 2: cdc acm dfu
                            // interface 3: cdc acm dfu subordinate interface
                            i == 0
                        } else {
                            true
                        }
                    } else {
                        false
                    }
                }
                _ => false,
            }
        })
        .collect();
    ports
}

fn stdin_thread(sender: Sender<()>) {
    loop {
        std::io::stdin().read_line(&mut String::new()).unwrap();
        if sender.blocking_send(()).is_err() {
            return;
        }
    }
}

#[tokio::main]
async fn main() -> ExitCode {
    // tracing_subscriber::fmt()
    //     .with_env_filter(
    //         EnvFilter::builder()
    //             .with_env_var("RUST_LOG")
    //             .with_default_directive(Level::INFO.into())
    //             .from_env_lossy(),
    //     )
    //     .init();
    let output_path = std::env::args().nth(1).expect("No output path");
    let port_arg = std::env::args().nth(2);
    let port_name = if let Some(port_name) = port_arg {
        port_name
    } else {
        let ports = get_ports();
        if ports.is_empty() {
            eprintln!("No device found, please connect one");
            return ExitCode::FAILURE;
        }
        if ports.len() > 1 {
            eprintln!("Mutiple devices found, please specify one");
            return ExitCode::FAILURE;
        };
        ports[0].port_name.clone()
    };

    eprintln!("Connecting to {}", port_name);
    let device = UsbDevice::connect_serial(&port_name, false).await.unwrap();
    eprintln!("Connected");

    let general_config = device.read_config().await.unwrap();
    eprintln!("Successfully read config");

    let (snd, mut stdin_line) = mpsc::channel(50);
    std::thread::spawn(move || stdin_thread(snd));
    eprintln!("Press enter to start recording");

    stdin_line.recv().await;
    eprintln!("Press enter to stop recording");

    let accel_stream = device
        .stream(ats_usb::packet::PacketType::AccelReport())
        .await
        .unwrap();
    let mot_stream = device
        .stream(ats_usb::packet::PacketType::ObjectReport())
        .await
        .unwrap();
    let comb_stream = device
        .stream(ats_usb::packet::PacketType::CombinedMarkersReport())
        .await
        .unwrap();
    let mut merged_stream = accel_stream.merge(mot_stream).merge(comb_stream);

    let mut packets = vec![];

    loop {
        tokio::select! {
            _ = stdin_line.recv() => break,
            pkt = merged_stream.next() => {
                let ts = std::time::SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis();
                packets.push((ts, pkt.unwrap()));
            }
        }
    }
    let mut bytes = vec![];
    general_config.serialize(&mut bytes);
    for (timestamp, packet_data) in packets.iter() {
        let packet = ats_usb::packet::Packet {
            data: packet_data.clone(),
            id: 0,
        };
        bytes.extend_from_slice(&timestamp.to_le_bytes());
        packet.serialize(&mut bytes);
    }

    eprintln!("Saving to {output_path}");
    std::fs::write(output_path, &bytes).expect("Failed to write output");

    ExitCode::SUCCESS
}
