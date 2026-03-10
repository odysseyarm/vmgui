//! Bond command — manually create a bond between a dongle and a device

use ats_usb::device::{MuxDevice, VmDevice};
use clap::Args;
use nusb::MaybeFuture as _;

#[derive(Args)]
pub struct BondArgs {
    /// Index of the dongle to use (use 'mux list-devices' to see available dongles)
    #[arg(short = 'm', long)]
    pub mux: Option<usize>,

    /// Index of the device to use (use 'device list-devices' to see available devices)
    #[arg(short = 'd', long)]
    pub device: Option<usize>,
}

fn list_muxes() -> Result<Vec<nusb::DeviceInfo>, String> {
    Ok(nusb::list_devices()
        .wait()
        .map_err(|e| format!("Failed to list USB devices: {e}"))?
        .into_iter()
        .filter(|info| info.vendor_id() == 0x1915 && info.product_id() == 0x5212)
        .collect())
}

async fn list_vm_devices() -> Result<Vec<nusb::DeviceInfo>, String> {
    let mut devices = Vec::new();
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
            Ok(_) => devices.push(info),
            Err(e) => eprintln!(
                "Skipping {:04X}:{:04X}: probe failed: {e}",
                info.vendor_id(),
                info.product_id()
            ),
        }
    }
    Ok(devices)
}

/// Parse the first 6 bytes of the FICR device ID from the USB serial number string.
/// The serial is 16 uppercase hex chars representing 8 bytes (low word then high word).
fn bd_addr_from_serial(serial: &str) -> Result<[u8; 6], String> {
    if serial.len() < 12 {
        return Err(format!("Serial number too short to derive BD address: {serial}"));
    }
    let mut addr = [0u8; 6];
    for i in 0..6 {
        addr[i] = u8::from_str_radix(&serial[i * 2..i * 2 + 2], 16)
            .map_err(|e| format!("Failed to parse serial number byte {i}: {e}"))?;
    }
    Ok(addr)
}

async fn pick_mux(index: Option<usize>) -> Result<(MuxDevice, [u8; 6]), String> {
    let muxes = list_muxes()?;
    if muxes.is_empty() {
        return Err("No dongle found (VID:0x1915 PID:0x5212)".to_string());
    }
    let info = match index {
        Some(i) => muxes
            .into_iter()
            .nth(i)
            .ok_or_else(|| format!("Dongle index {i} not found"))?,
        None => {
            if muxes.len() > 1 {
                return Err(format!(
                    "Multiple dongles found ({}). Specify one with --mux <index>.",
                    muxes.len()
                ));
            }
            muxes.into_iter().next().unwrap()
        }
    };
    let serial = info
        .serial_number()
        .ok_or_else(|| "Dongle has no USB serial number".to_string())?;
    let dongle_bd_addr = bd_addr_from_serial(serial)?;
    let mux = MuxDevice::connect_usb(info)
        .await
        .map_err(|e| format!("Failed to connect to dongle: {e}"))?;
    Ok((mux, dongle_bd_addr))
}

async fn pick_device(index: Option<usize>) -> Result<VmDevice, String> {
    let devs = list_vm_devices().await?;
    if devs.is_empty() {
        return Err("No device found.".to_string());
    }
    let info = match index {
        Some(i) => devs
            .into_iter()
            .nth(i)
            .ok_or_else(|| format!("Device index {i} not found"))?,
        None => {
            if devs.len() > 1 {
                return Err(format!(
                    "Multiple devices found ({}). Specify one with --device <index>.",
                    devs.len()
                ));
            }
            devs.into_iter().next().unwrap()
        }
    };
    VmDevice::connect_usb_any_mode(info)
        .await
        .map_err(|e| format!("Failed to connect to device: {e}"))
}

fn format_addr(addr: &[u8; 6]) -> String {
    format!(
        "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
    )
}

pub async fn handle_bond(args: BondArgs) -> Result<(), String> {
    use rand::RngCore as _;

    let (mux, dongle_bd_addr) = pick_mux(args.mux).await?;
    let device = pick_device(args.device).await?;

    // Read device UUID — this is also its BLE address
    let device_bd_addr: [u8; 6] = device
        .read_uuid()
        .await
        .map_err(|e| format!("Failed to read device UUID: {e}"))?;

    println!("Dongle BD addr: {}", format_addr(&dongle_bd_addr));
    println!("Device BD addr: {}", format_addr(&device_bd_addr));

    // Generate random LTK and IRK (same values for both sides)
    let mut rng = rand::thread_rng();
    let mut ltk = [0u8; 16];
    let mut irk = [0u8; 16];
    rng.fill_bytes(&mut ltk);
    rng.fill_bytes(&mut irk);

    // Dongle bond: bd_addr = device (who the dongle connects to)
    let dongle_entry = protodongers::control::BondEntry {
        bd_addr: device_bd_addr,
        ltk,
        security_level: 2, // EncryptedAuthenticated
        is_bonded: true,
        irk: Some(irk),
    };

    // Device bond: bd_addr = dongle (who the device expects to connect to it)
    let device_entry = protodongers::control::BondEntry {
        bd_addr: dongle_bd_addr,
        ltk,
        security_level: 2,
        is_bonded: true,
        irk: Some(irk),
    };

    println!("Sending bond to dongle...");
    mux.add_bond(dongle_entry)
        .await
        .map_err(|e| format!("Failed to add bond to dongle: {e}"))?;
    println!("Dongle: bond added");

    println!("Sending bond to device...");
    device
        .add_bond(device_entry)
        .await
        .map_err(|e| format!("Failed to add bond to device: {e}"))?;
    println!("Device: bond added");

    println!(
        "Bond created successfully: dongle {} <-> device {}",
        format_addr(&dongle_bd_addr),
        format_addr(&device_bd_addr)
    );
    Ok(())
}
