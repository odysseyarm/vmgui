use nusb::MaybeFuture as _;

fn main() {
    let devices: Vec<_> = nusb::list_devices()
        .wait()
        .unwrap()
        .into_iter()
        .filter(|info| {
            info.vendor_id() == 0x1915
                && info.product_id() == 0x5212
        })
        .collect();

    println!("Found {} device(s)", devices.len());
    for (i, dev) in devices.iter().enumerate() {
        println!("\nDevice {}: VID={:04x} PID={:04x}", i, dev.vendor_id(), dev.product_id());
        println!("  Bus: {:?}, Address: {}", dev.bus_id(), dev.device_address());
        println!("  Interfaces:");
        for iface in dev.interfaces() {
            println!("    Interface {}: class={:02x} subclass={:02x}", 
                iface.interface_number(), 
                iface.class(),
                iface.subclass());
        }
    }
}
