use std::time::Duration;

use crate::packet::{Packet, Register, Port};

mod packet;

fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let mut serial_port = serialport::new(&args[1], 115200).timeout(Duration::from_secs(3)).open().unwrap();

    let registers = [
        (0x00, 0x02), // product ID
        (0x00, 0x03), //
        (0x00, 0x0f), // DSP noise threshold
        (0x00, 0x0b), // DSP max area threshold
        (0x00, 0x0c), //
        (0x00, 0x10), // DSP orientation ratio
        (0x00, 0x11), // DSP orientation factor
        (0x00, 0x19), // DSP maximum object number
        (0x01, 0x05), // sensor gain 1
        (0x01, 0x06), // sensor gain 2
        (0x01, 0x0e), // sensor exposure length
        (0x01, 0x0f), //
        (0x0c, 0x60), // interpolated resolution x
        (0x0c, 0x61), //
        (0x0c, 0x62), // interpolated resolution y
        (0x0c, 0x63), //
        (0x0c, 0x07), // frame period
        (0x0c, 0x08), //
        (0x0c, 0x09), //
    ];

    let mut buf = Vec::new();
    for (bank, address) in registers {
        println!("Reading bank {bank:x}, addr {address:x}");
        let pkt = Packet::ReadRegister(Register {
            port: Port::Nv,
            bank,
            address,
        });
        buf.clear();
        buf.push(0xff);
        pkt.serialize(&mut buf);
        // println!("Write {buf:?}");
        serial_port.write_all(&buf).unwrap();
        serial_port.flush().unwrap();
        buf.clear();
        buf.resize(10, 0);
        let bytes_read = serial_port.read(&mut buf).unwrap();
        let resp = Packet::parse(&mut &buf[..bytes_read]);
        // println!("Read {bytes_read} bytes: {:x?}", &buf[..bytes_read]);
        println!("{resp:x?}");
    }
}
