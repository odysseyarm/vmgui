use std::net::{UdpSocket, Ipv4Addr};
use vision_module_gui::packet::{Packet, PacketData, PacketType, Port, Register, StreamChoice, StreamUpdate};
use vision_module_gui::device::decode_slip_frame;

fn main() {
    let client = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 23456)).unwrap();
    client.set_broadcast(true).unwrap();
    client.send_to(&[255, 1], ("10.0.0.255", 23456)).unwrap();

    // let mut udp_packet = vec![1, 0, 255];
    // let pkt = Packet {
    //     id: 0,
    //     data: PacketData::ReadRegister(Register { port: Port::Wf, bank: 0x00, address: 0x02 }),
    // };
    // pkt.serialize(&mut udp_packet);
    // client.send_to(&udp_packet, ("10.0.0.255", 23456)).unwrap();

    // let mut udp_packet = vec![1, 0, 255];
    // let pkt = Packet {
    //     id: 1,
    //     data: PacketData::StreamUpdate(StreamUpdate { mask: 0b0100 | 0b0010, active: true })
    // };
    // pkt.serialize(&mut udp_packet);
    // client.send_to(&udp_packet, ("10.0.0.255", 23456)).unwrap();

    let mut data = vec![0; 1472];
    let mut buf = vec![];
    let mut slip_buf = vec![];
    let mut total_accel_samples = 0;
    let start_time = std::time::Instant::now();
    let mut total_combined_marker_samples = 0;

    loop {
        let resp = client.recv_from(&mut data);
        if let Ok((len, addr)) = resp {
            if data[1] != 0 {
                println!("Received {:?} from {addr}", &buf[..]);
                continue;
            }
            // use slip_decoder with slip_buf and data[2..len] to get the actual data to put in buf
            slip_buf.extend_from_slice(&data[2..len]);
            if let Ok(()) = decode_slip_frame(&mut slip_buf) {
                buf.extend_from_slice(&slip_buf);
                slip_buf.clear();
            } else {
                // slip_buf.clear();
                continue;
            }
            // if (len == 0 || len == 1) {
            //     continue;
            // }
            // let [words1, words2, ty, id, ..] = data[2..len] else {
            //     println!("Weirdness");
            //     continue;
            // };
            // let words = u16::from_le_bytes([words1, words2]);
            // let ty = PacketType::try_from(ty);

            // let _len = usize::from(words)*2;
            println!("{}", buf.len());
            // println!("{:?}", &data[2..len]);
            // println!("Received {:?} from {addr}", &buf[..buf.len()]);
            let buf_slice = &mut &buf[..];
            if let Ok(pkt) = Packet::parse(buf_slice) {
                // buf.drain(0..buf.len() - buf_slice.len());
                buf.clear();

                // if len > 2 + _len {
                //     println!("Leftover {:?} from {addr}", &data[2 + _len..len]);
                // }
                println!("Received {:?} from {addr}", pkt);
                match pkt.data {
                    PacketData::AccelReport(_) => {
                        let elapsed = start_time.elapsed().as_secs_f64();
                        total_accel_samples += 1;
                        let accel_hz_avg = 1. / (elapsed / total_accel_samples as f64);
                        println!("Accel: {:?} Hz", accel_hz_avg);
                    }
                    PacketData::CombinedMarkersReport(_) => {
                        let elapsed = start_time.elapsed().as_secs_f64();
                        total_combined_marker_samples += 1;
                        let combined_marker_hz_avg = 1. / (elapsed / total_combined_marker_samples as f64);
                        println!("Combined Marker: {:?} Hz", combined_marker_hz_avg);
                    }
                    // Malformy
                    _ => { buf.clear(); }
                }
            } else {
                println!("Received {:?} from {addr}", &buf[..]);
                // buf.clear();
            }
        }
    }
}
