use std::{process::ExitCode, time::UNIX_EPOCH};

use ats_usb::device::VmDevice;
use nusb::{DeviceInfo, MaybeFuture as _};
use protodongers::control::device::TransportMode;
use tokio::sync::mpsc::{self, Sender};
use tokio_stream::StreamExt;
#[allow(unused_imdevices)]
use tracing::Level;
#[allow(unused_imdevices)]
use tracing_subscriber::EnvFilter;

async fn get_devices() -> Vec<DeviceInfo> {
    let devices = match nusb::list_devices().wait() {
        Ok(devs) => devs,
        Err(e) => {
            eprintln!("Failed to list USB devices: {e}");
            return vec![];
        }
    };

    let mut filtered = Vec::new();
    for info in devices.into_iter().filter(|info| {
        info.vendor_id() == 0x1915
            && (info.product_id() == 0x520F || info.product_id() == 0x5210)
    }) {
        match VmDevice::probe_transport_mode(&info).await {
            Ok(TransportMode::Usb) => filtered.push(info),
            Ok(mode) => eprintln!(
                "Skipping {:04x}:{:04x} - transport mode {:?}",
                info.vendor_id(),
                info.product_id(),
                mode
            ),
            Err(e) => eprintln!(
                "Skipping {:04x}:{:04x} - failed to probe mode: {e}",
                info.vendor_id(),
                info.product_id()
            ),
        }
    }

    filtered
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
    let device_info = if port_arg.is_some() {
        eprintln!("Port argument is no longer supported, using first available device");
        let devices = get_devices().await;
        if devices.is_empty() {
            eprintln!("No device in USB mode found, please connect one");
            return ExitCode::FAILURE;
        }
        devices.into_iter().next().unwrap()
    } else {
        let devices = get_devices().await;
        if devices.is_empty() {
            eprintln!("No device in USB mode found, please connect one");
            return ExitCode::FAILURE;
        }
        if devices.len() > 1 {
            eprintln!("Multiple devices found, using first one");
        };
        devices.into_iter().next().unwrap()
    };

    eprintln!("Connecting to device");
    let device = VmDevice::connect_usb(device_info).await.unwrap();
    eprintln!("Connected");

    let general_config = device.read_all_config().await.unwrap();
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
    postcard::to_slice(&general_config, &mut bytes).unwrap();
    for (timestamp, packet_data) in packets.iter() {
        let packet = ats_usb::packets::vm::Packet {
            data: packet_data.clone(),
            id: 0,
        };
        bytes.extend_from_slice(&timestamp.to_le_bytes());
        postcard::to_slice(&packet, &mut bytes).unwrap();
    }

    eprintln!("Saving to {output_path}");
    std::fs::write(output_path, &bytes).expect("Failed to write output");

    ExitCode::SUCCESS
}