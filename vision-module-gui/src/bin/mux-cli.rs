//! Mux CLI tool for managing the dongle

use std::process::ExitCode;

use ats_usb::device::MuxDevice;
use clap::{Parser, Subcommand};
use nusb::MaybeFuture as _;

#[derive(Parser)]
#[command(name = "mux-cli")]
#[command(about = "CLI tool for managing the dongle", long_about = None)]
struct Cli {
    /// Index of the device to use (use 'devices' command to list)
    #[arg(short, long)]
    device: Option<usize>,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Print USB interfaces and endpoints for the dongle
    UsbInfo,
    /// List all available dongles
    Devices,
    /// List currently connected BLE devices
    List,
    /// Read firmware version
    Version,
    /// Start pairing mode with optional timeout (ms)
    Pair {
        #[arg(short, long, default_value_t = 120000)]
        timeout: u32,
    },
    /// Cancel pairing mode
    PairCancel,
    /// Clear bonds
    Clear,
}

fn list_dongles() -> Result<Vec<nusb::DeviceInfo>, String> {
    let devices: Vec<_> = nusb::list_devices()
        .wait()
        .map_err(|e| format!("Failed to list USB devices: {e}"))?
        .into_iter()
        .filter(|info| info.vendor_id() == 0x1915 && info.product_id() == 0x5210)
        .collect();

    Ok(devices)
}

async fn connect_to_device(device_index: Option<usize>) -> Result<MuxDevice, String> {
    let devices = list_dongles()?;

    if devices.is_empty() {
        return Err(
            "No dongle found (VID:0x1915 PID:0x5210). Please connect a dongle.".to_string(),
        );
    }

    let device_info = if let Some(idx) = device_index {
        devices
            .get(idx)
            .ok_or_else(|| {
                format!(
                    "Device index {} not found. Use 'devices' command to see available devices.",
                    idx
                )
            })?
            .clone()
    } else {
        if devices.len() > 1 {
            return Err(format!(
                "Multiple dongles found ({}). Please specify which one to use with --device <index>. Use 'devices' command to list them.",
                devices.len()
            ));
        }
        devices[0].clone()
    };

    MuxDevice::connect_usb(device_info)
        .await
        .map_err(|e| format!("Failed to connect to dongle: {e}"))
}

fn format_device_addr(addr: &[u8; 6]) -> String {
    format!(
        "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
    )
}

async fn cmd_devices() -> Result<(), String> {
    let devices = list_dongles()?;

    if devices.is_empty() {
        println!("No dongles found (VID:0x1915 PID:0x5210)");
    } else {
        println!("Available dongles:");
        for (i, info) in devices.iter().enumerate() {
            let product = info
                .product_string()
                .map(|s| s.to_string())
                .unwrap_or_else(|| "Unknown".to_string());
            println!(
                "  [{}] VID:0x{:04X} PID:0x{:04X} - {}",
                i,
                info.vendor_id(),
                info.product_id(),
                product
            );
        }
        println!();
        println!("Use --device <index> to select a specific dongle");
    }
    Ok(())
}

async fn cmd_list(device: &MuxDevice) -> Result<(), String> {
    let devices = device
        .request_devices()
        .await
        .map_err(|e| format!("Failed to get devices: {e}"))?;

    if devices.is_empty() {
        println!("No devices currently connected");
    } else {
        println!("Connected devices ({}):", devices.len());
        for dev in devices.iter() {
            println!("  {}", format_device_addr(dev));
        }
    }
    Ok(())
}

async fn cmd_version(device: &MuxDevice) -> Result<(), String> {
    let version = device
        .read_version()
        .await
        .map_err(|e| format!("Failed to read version: {e}"))?;

    println!(
        "Firmware version: {}.{}.{}",
        version.firmware_semver[0], version.firmware_semver[1], version.firmware_semver[2]
    );
    println!(
        "Protocol version: {}.{}.{}",
        version.protocol_semver[0], version.protocol_semver[1], version.protocol_semver[2]
    );
    Ok(())
}

#[tokio::main]
async fn main() -> ExitCode {
    let cli = Cli::parse();

    let result = match &cli.command {
        Commands::Devices => cmd_devices().await,
        Commands::List => {
            let device = match connect_to_device(cli.device).await {
                Ok(dev) => dev,
                Err(e) => {
                    eprintln!("Error: {e}");
                    return ExitCode::FAILURE;
                }
            };
            cmd_list(&device).await
        }
        Commands::Version => {
            let device = match connect_to_device(cli.device).await {
                Ok(dev) => dev,
                Err(e) => {
                    eprintln!("Error: {e}");
                    return ExitCode::FAILURE;
                }
            };
            cmd_version(&device).await
        }
        Commands::Pair { timeout } => {
            let device = match connect_to_device(cli.device).await {
                Ok(dev) => dev,
                Err(e) => {
                    eprintln!("Error: {e}");
                    return ExitCode::FAILURE;
                }
            };
            cmd_pair(&device, *timeout).await
        }
        Commands::PairCancel => {
            let device = match connect_to_device(cli.device).await {
                Ok(dev) => dev,
                Err(e) => {
                    eprintln!("Error: {e}");
                    return ExitCode::FAILURE;
                }
            };
            cmd_pair_cancel(&device).await
        }
        Commands::UsbInfo => cmd_usb_info(cli.device).await,
        Commands::Clear => {
            let device = match connect_to_device(cli.device).await {
                Ok(dev) => dev,
                Err(e) => {
                    eprintln!("Error: {e}");
                    return ExitCode::FAILURE;
                }
            };
            cmd_clear(&device).await
        }
    };

    match result {
        Ok(_) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Error: {e}");
            ExitCode::FAILURE
        }
    }
}

async fn cmd_pair(device: &MuxDevice, timeout: u32) -> Result<(), String> {
    println!("Starting pairing for {} ms...", timeout);
    device
        .start_pairing(timeout)
        .await
        .map_err(|e| format!("Failed to start pairing: {e}"))?;
    println!("Pairing started. Waiting for result...");
    match device
        .wait_pairing_event()
        .await
        .map_err(|e| format!("Error waiting for pairing event: {e}"))?
    {
        ats_usb::device::PairingEvent::Result(addr) => {
            println!("Paired with device {}", format_device_addr(&addr));
            Ok(())
        }
        ats_usb::device::PairingEvent::Timeout => {
            println!("Pairing timed out");
            Ok(())
        }
        ats_usb::device::PairingEvent::Cancelled => {
            println!("Pairing cancelled");
            Ok(())
        }
    }
}

async fn cmd_pair_cancel(device: &MuxDevice) -> Result<(), String> {
    println!("Cancelling pairing mode...");
    device
        .cancel_pairing()
        .await
        .map_err(|e| format!("Failed to cancel pairing: {e}"))?;
    println!("Pairing cancelled");
    Ok(())
}

async fn cmd_clear(device: &MuxDevice) -> Result<(), String> {
    println!("Clearing bonds...");
    device
        .clear_bonds()
        .await
        .map_err(|e| format!("Failed to clear bonds: {e}"))?;
    println!("Bonds cleared");
    Ok(())
}

async fn cmd_usb_info(device_index: Option<usize>) -> Result<(), String> {
    use nusb::transfer::{Bulk, Interrupt, In, Out};
    let devices = list_dongles()?;
    if devices.is_empty() {
        return Err("No dongle found (VID:0x1915 PID:0x5210)".to_string());
    }
    let device_info = if let Some(idx) = device_index {
        devices.get(idx).cloned().ok_or_else(|| format!(
            "Device index {} not found. Use 'devices' to list.", idx
        ))?
    } else {
        if devices.len() > 1 {
            return Err(format!(
                "Multiple dongles found ({}). Please specify with --device <index>.",
                devices.len()
            ));
        }
        devices[0].clone()
    };
    let dev = device_info
        .open()
        .await
        .map_err(|e| format!("Failed to open device: {e}"))?;
    println!(
        "Probing interfaces for VID:0x{:04X} PID:0x{:04X}",
        device_info.vendor_id(),
        device_info.product_id()
    );
    for idx in 0u8..8 {
        match dev.claim_interface(idx).await {
            Ok(iface) => {
                let bulk_in = iface.endpoint::<Bulk, In>(0x81).is_ok();
                let bulk_out = iface.endpoint::<Bulk, Out>(0x01).is_ok();
                let intr_in = iface.endpoint::<Interrupt, In>(0x86).is_ok();
                println!(
                    "  iface {}: bulk_out 0x01={}, bulk_in 0x81={}, intr_in 0x86={}",
                    idx, bulk_out, bulk_in, intr_in
                );
            }
            Err(_) => {}
        }
    }
    Ok(())
}
