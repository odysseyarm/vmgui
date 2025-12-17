//! Direct device (lite/vm) management commands

use ats_usb::device::VmDevice;
use protodongers::control::device::TransportMode;
use clap::Subcommand;
use nusb::MaybeFuture as _;

#[derive(Subcommand)]
pub enum DeviceCommands {
    /// List all available devices
    ListDevices,
    /// Read firmware version
    Version,
    /// Start pairing mode with optional timeout (ms)
    Pair {
        #[arg(short, long, default_value_t = 120000)]
        timeout: u32,
    },
    /// Cancel pairing mode
    PairCancel,
    /// Clear bond (for lite devices with single bond)
    ClearBond,
    /// Calibrate accelerometer
    AccelCalib {
        /// Number of samples per orientation
        #[arg(short, default_value_t = 400)]
        samples: usize,
        /// Gravity value
        #[arg(short, default_value_t = 9.80665)]
        gravity: f64,
        /// Output JSON file
        output: String,
    },
    /// Calibrate gyroscope
    GyroCalib {
        /// Number of samples
        #[arg(short, default_value_t = 400)]
        samples: usize,
        /// Output JSON file
        output: String,
    },
    /// Stream accelerometer data
    Stream,
}

async fn list_devices(require_usb_mode: bool) -> Result<Vec<nusb::DeviceInfo>, String> {
    let mut devices = Vec::new();
    let mut skipped = 0usize;
    for info in nusb::list_devices()
        .wait()
        .map_err(|e| format!("Failed to list USB devices: {e}"))?
        .into_iter()
        .filter(|info| {
            info.vendor_id() == 0x1915
                && (info.product_id() == 0x520F
                    || info.product_id() == 0x5210
                    || info.product_id() == 0x5211)
        })
    {
        match VmDevice::probe_transport_mode(&info).await {
            Ok(mode) => {
                if require_usb_mode && !matches!(mode, TransportMode::Usb) {
                    skipped += 1;
                    continue;
                }
                if !matches!(mode, TransportMode::Usb) {
                    eprintln!(
                        "Info: {:04X}:{:04X} is in {:?} mode",
                        info.vendor_id(),
                        info.product_id(),
                        mode
                    );
                }
                devices.push(info);
            }
            Err(e) => {
                skipped += 1;
                eprintln!(
                    "Skipping device {:04X}:{:04X}: transport probe failed: {e}",
                    info.vendor_id(),
                    info.product_id()
                );
            }
        }
    }
    if require_usb_mode && devices.is_empty() && skipped > 0 {
        eprintln!("Found {skipped} device(s) not in USB mode; no USB-mode devices available");
    }
    Ok(devices)
}

async fn connect_to_device(
    device_index: Option<usize>,
    require_usb_mode: bool,
) -> Result<VmDevice, String> {
    let devices = list_devices(require_usb_mode).await?;

    if devices.is_empty() {
        return Err("No device found. Please connect a device.".to_string());
    }

    let device_info = if let Some(idx) = device_index {
        devices
            .get(idx)
            .ok_or_else(|| {
                format!(
                    "Device index {} not found. Use 'device list-devices' to see available devices.",
                    idx
                )
            })?
            .clone()
    } else {
        if devices.len() > 1 {
            return Err(format!(
                "Multiple devices found ({}). Please specify which one to use with --device <index>. Use 'device list-devices' to list them.",
                devices.len()
            ));
        }
        devices[0].clone()
    };

    if require_usb_mode {
        VmDevice::connect_usb(device_info)
            .await
            .map_err(|e| format!("Failed to connect to device: {e}"))
    } else {
        VmDevice::connect_usb_any_mode(device_info)
            .await
            .map_err(|e| format!("Failed to connect to device: {e}"))
    }
}

async fn cmd_list_devices() -> Result<(), String> {
    let devices = list_devices(false).await?;

    if devices.is_empty() {
        println!("No devices found");
    } else {
        println!("Available devices:");
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
            println!("      Bus: {:?}, Address: {}", info.bus_id(), info.device_address());
            println!("      Interfaces:");
            for iface in info.interfaces() {
                println!(
                    "        Interface {}: class=0x{:02X} subclass=0x{:02X}",
                    iface.interface_number(),
                    iface.class(),
                    iface.subclass()
                );
            }
        }
        println!();
        println!("Use --device <index> to select a specific device");
    }
    Ok(())
}

async fn cmd_version(device: &VmDevice) -> Result<(), String> {
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

async fn cmd_pair(device: &VmDevice, timeout: u32) -> Result<(), String> {
    println!("Starting pairing mode (timeout: {}ms)...", timeout);
    
    match device.start_pairing(timeout).await {
        Ok(()) => {
            println!("Pairing mode started, waiting for device...");
            
            match device.wait_pairing_event().await {
                Ok(addr) => {
                    println!("Successfully paired with device: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
                    Ok(())
                }
                Err(e) => Err(format!("Pairing failed: {}", e)),
            }
        }
        Err(e) => Err(format!("Failed to start pairing: {}", e)),
    }
}

async fn cmd_pair_cancel(device: &VmDevice) -> Result<(), String> {
    match device.cancel_pairing().await {
        Ok(()) => {
            println!("Pairing cancelled");
            Ok(())
        }
        Err(e) => Err(format!("Failed to cancel pairing: {}", e)),
    }
}

async fn cmd_clear_bond(device: &VmDevice) -> Result<(), String> {
    match device.clear_bond().await {
        Ok(()) => {
            println!("Bond cleared successfully");
            Ok(())
        }
        Err(e) => Err(format!("Failed to clear bond: {}", e)),
    }
}
pub async fn handle_command(
    device_index: Option<usize>,
    command: DeviceCommands,
) -> Result<(), String> {
    match command {
        DeviceCommands::ListDevices => cmd_list_devices().await,
        DeviceCommands::Version => {
            let device = connect_to_device(device_index, true).await?;
            cmd_version(&device).await
        }
        DeviceCommands::Pair { timeout } => {
            let device = connect_to_device(device_index, false).await?;
            cmd_pair(&device, timeout).await
        }
        DeviceCommands::PairCancel => {
            let device = connect_to_device(device_index, false).await?;
            cmd_pair_cancel(&device).await
        }
        DeviceCommands::ClearBond => {
            let device = connect_to_device(device_index, false).await?;
            cmd_clear_bond(&device).await
        }
        DeviceCommands::AccelCalib {
            samples,
            gravity,
            output,
        } => {
            let mut device = connect_to_device(device_index, true).await?;
            crate::calibration::cmd_accel_calib(&mut device, samples, gravity, &output).await
        }
        DeviceCommands::GyroCalib { samples, output } => {
            let mut device = connect_to_device(device_index, true).await?;
            crate::calibration::cmd_gyro_calib(&mut device, samples, &output).await
        }
        DeviceCommands::Stream => {
            let mut device = connect_to_device(device_index, true).await?;
            crate::calibration::cmd_stream(&mut device).await
        }
    }
}
