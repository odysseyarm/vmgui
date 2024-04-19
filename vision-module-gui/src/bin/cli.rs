use std::process::ExitCode;

use ats_usb::{device::UsbDevice, packet::Port};

fn print_help() {
    eprintln!("Usage: ./cli [options] <address>");
    eprintln!();
    eprintln!("Options:");
    eprintln!("    -s  Connect over serial");
    eprintln!("    -t  Connect over tcp");
    eprintln!("    -u  Connect over udp");
}

#[tokio::main]
async fn main() -> ExitCode {
    let args = std::env::args().collect::<Vec<_>>();
    if args.len() < 3 {
        print_help();
        return ExitCode::FAILURE;
    }
    let device = match &*args[1] {
        "-s" => UsbDevice::connect_serial(&args[2]).await.unwrap(),
        "-t" => UsbDevice::connect_tcp(&args[2]).unwrap(),
        "-u" => UsbDevice::connect_hub("0.0.0.0:0", &args[2]).await.unwrap(),
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
