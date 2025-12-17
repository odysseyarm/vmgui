use std::process::ExitCode;

use ats_usb::{
    device::VmDevice,
    packets::vm::Port,
};
use nusb::MaybeFuture as _;
use protodongers::control::device::TransportMode;
fn print_help() {
    eprintln!("Usage: ./cli <usb>");
    eprintln!();
    eprintln!("Connect to first available USB device and read product IDs");
}

#[tokio::main]
async fn main() -> ExitCode {
    let args = std::env::args().collect::<Vec<_>>();
    if args.len() < 2 {
        print_help();
        return ExitCode::FAILURE;
    }

    let device = match &*args[1] {
        "usb" => {
            // Find first available device in USB mode
            let devices = nusb::list_devices().wait().unwrap();
            let mut usb_mode_devices = Vec::new();

            for info in devices.into_iter().filter(|info| {
                info.vendor_id() == 0x1915
                    && (info.product_id() == 0x520F
                        || info.product_id() == 0x5210
                        || info.product_id() == 0x5211)
            }) {
                match VmDevice::probe_transport_mode(&info).await {
                    Ok(TransportMode::Usb) => usb_mode_devices.push(info),
                    Ok(mode) => {
                        eprintln!(
                            "Skipping {:04x}:{:04x} - transport mode {:?}",
                            info.vendor_id(),
                            info.product_id(),
                            mode
                        );
                    }
                    Err(e) => {
                        eprintln!(
                            "Skipping {:04x}:{:04x} - failed to probe mode: {e}",
                            info.vendor_id(),
                            info.product_id()
                        );
                    }
                }
            }

            if usb_mode_devices.is_empty() {
                eprintln!("No device in USB mode found");
                return ExitCode::FAILURE;
            }

            VmDevice::connect_usb(usb_mode_devices.into_iter().next().unwrap())
                .await
                .unwrap()
        }
        _ => {
            print_help();
            return ExitCode::FAILURE;
        }
    };

    let nf_pid = device.product_id(Port::Nf).await.unwrap();
    let wf_pid = device.product_id(Port::Wf).await.unwrap();
    eprintln!("Nearfield Product ID: 0x{nf_pid:x}");
    eprintln!("Widefield Product ID: 0x{wf_pid:x}");
    ExitCode::SUCCESS
}