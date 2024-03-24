use std::net::{UdpSocket, Ipv4Addr};
use vision_module_gui::packet::{Packet, PacketData, Register, Port};

fn main() {
    let client = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 23456)).unwrap();
    client.set_broadcast(true).unwrap();
    client.send_to(&[255, 1], ("10.0.0.255", 23456)).unwrap();
    let mut udp_packet = vec![1, 0, 255];
    let pkt = Packet {
        id: 0,
        data: PacketData::ReadRegister(Register { port: Port::Wf, bank: 0x00, address: 0x02 }),
    };
    pkt.serialize(&mut udp_packet);
    client.send_to(&udp_packet, ("10.0.0.255", 23456)).unwrap();
    let mut data = vec![0; 1472];
    loop {
        let resp = client.recv_from(&mut data);
        if let Ok((len, addr)) = resp {
            if let Ok(pkt) = Packet::parse(&mut &data[2..len]) {
                println!("Received {:?} from {addr}", pkt);
            } else {
                println!("Received {:?} from {addr}", &data[..len]);
            }
        }
    }
}
