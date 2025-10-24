use std::process::ExitCode;

use ats_usb::{device::VmDevice, packets::vm::Port};
use nusb::MaybeFuture as _;

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
            // Find first available device
            let devices: Vec<_> = nusb::list_devices()
                .wait()
                .unwrap()
                .into_iter()
                .filter(|info| {
                    info.vendor_id() == 0x1915
                        && (info.product_id() == 0x520F
                            || info.product_id() == 0x5210
                            || info.product_id() == 0x5211)
                })
                .collect();

            if devices.is_empty() {
                eprintln!("No device found");
                return ExitCode::FAILURE;
            }

            VmDevice::connect_usb(devices.into_iter().next().unwrap())
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
