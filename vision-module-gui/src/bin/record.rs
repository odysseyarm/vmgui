use std::{process::ExitCode, time::UNIX_EPOCH};

use ats_usb::{device::UsbDevice, packets::vm::Serialize as _};
use nusb::{DeviceInfo, MaybeFuture as _};
use tokio::sync::mpsc::{self, Sender};
use tokio_stream::StreamExt;
#[allow(unused_imdevices)]
use tracing::Level;
#[allow(unused_imdevices)]
use tracing_subscriber::EnvFilter;

fn get_devices() -> Vec<DeviceInfo> {
    let devices = nusb::list_devices().wait();
    let devices = devices
        .unwrap()
        .into_iter()
        .filter(|info| {
            info.vendor_id() == 0x1915 && info.product_id() == 0x520F || info.product_id() == 0x5210
        })
        .collect();
    devices
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
        let devices = get_devices();
        if devices.is_empty() {
            eprintln!("No device found, please connect one");
            return ExitCode::FAILURE;
        }
        if devices.len() > 1 {
            eprintln!("Mutiple devices found, please specify one");
            return ExitCode::FAILURE;
        };
        devices[0].port_name.clone()
    };

    eprintln!("Connecting to {}", port_name);
    let device = UsbDevice::connect(&port_name).await.unwrap();
    eprintln!("Connected");

    let general_config = device.read_config().await.unwrap();
    eprintln!("Successfully read config");

    let (snd, mut stdin_line) = mpsc::channel(50);
    std::thread::spawn(move || stdin_thread(snd));
    eprintln!("Press enter to start recording");

    stdin_line.recv().await;
    eprintln!("Press enter to stop recording");

    let accel_stream = device
        .stream(ats_usb::packets::vm::PacketType::AccelReport())
        .await
        .unwrap();
    let mot_stream = device
        .stream(ats_usb::packets::vm::PacketType::ObjectReport())
        .await
        .unwrap();
    let comb_stream = device
        .stream(ats_usb::packets::vm::PacketType::CombinedMarkersReport())
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
    postcard::to_slice(&general_config, &mut bytes);
    for (timestamp, packet_data) in packets.iter() {
        let packet = ats_usb::packets::vm::Packet {
            data: packet_data.clone(),
            id: 0,
        };
        bytes.extend_from_slice(&timestamp.to_le_bytes());
        postcard::to_slice(&packet, &mut bytes);
    }

    eprintln!("Saving to {output_path}");
    std::fs::write(output_path, &bytes).expect("Failed to write output");

    ExitCode::SUCCESS
}
