use ats_usb::packets::vm::{Packet, PacketData, Port, Register};
use std::net::{Ipv4Addr, SocketAddr};
// use multicast_socket::MulticastSocket;
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::net::UdpSocket;

fn main() {
    let local_addr = SockAddr::from(SocketAddr::new(Ipv4Addr::UNSPECIFIED.into(), 0));

    // todo MulticastSocket has to be turned into Ext for std, socket2, and tokio
    // let client = MulticastSocket::all_interfaces(multicast_addr_std).unwrap();

    let client =
        Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP)).expect("ipv4 dgram socket");

    client.set_broadcast(true).unwrap();

    client.bind(&local_addr).unwrap();

    let broadcast_addr = SockAddr::from(SocketAddr::new(
        std::net::IpAddr::V4(Ipv4Addr::BROADCAST),
        23456,
    ));
    client.send_to(&[255, 1], &broadcast_addr).unwrap();

    let mut udp_packet = vec![1, 0, 255];
    let pkt = Packet {
        id: 0,
        data: PacketData::ReadRegister(Register {
            port: Port::Wf,
            bank: 0x00,
            address: 0x02,
        }),
    };
    pkt.serialize(&mut udp_packet);
    client.send_to(&udp_packet, &broadcast_addr).unwrap();

    // let mut udp_packet = vec![1, 0, 255];
    // let pkt = Packet {
    //     id: 1,
    //     data: PacketData::StreamUpdate(StreamUpdate { mask: 0b0100 | 0b0010, active: true })
    // };
    // pkt.serialize(&mut udp_packet);
    // client.send_to(&udp_packet, &multicast_addr).unwrap();

    let client: UdpSocket = client.into();

    let mut data = vec![0; 1472];
    let mut total_accel_samples = 0;
    let start_time = std::time::Instant::now();
    let mut total_combined_marker_samples = 0;

    fn process_one(
        start_time: std::time::Instant,
        data: &mut Vec<u8>,
        _total_accel_samples: &mut u64,
        total_combined_marker_samples: &mut u64,
        addr: SocketAddr,
    ) {
        match Packet::parse(&mut data.as_slice()) {
            Ok(pkt) => {
                println!("Received {:?} from {addr}", pkt);
                match pkt.data {
                    PacketData::CombinedMarkersReport(_) => {
                        let elapsed = start_time.elapsed().as_secs_f64();
                        *total_combined_marker_samples += 1;
                        let combined_marker_hz_avg =
                            1. / (elapsed / *total_combined_marker_samples as f64);
                        println!("Combined Marker: {:?} Hz", combined_marker_hz_avg);
                    }
                    _ => {}
                }
            }
            Err(e) => match e {
                ats_usb::packets::vm::Error::UnexpectedEof { .. } => {}
                ats_usb::packets::vm::Error::UnrecognizedPacketId(_) => {}
                ats_usb::packets::vm::Error::UnrecognizedPort => {}
                _ => {
                    println!("Error: {:?}", e);
                }
            },
        }
    }

    let mut cobs_buf: Vec<u8> = Vec::with_capacity(4096);

    loop {
        let resp = client.recv_from(&mut data);
        if let Ok((len, addr)) = resp {
            let chunk = &data[..len];
            println!("Received {:?} from {addr}", chunk);

            // Accumulate incoming bytes; may contain partial or multiple frames
            cobs_buf.extend_from_slice(chunk);

            // Process all complete COBS frames (0x00-delimited)
            while let Some(end_idx) = cobs_buf.iter().position(|&b| b == 0x00) {
                // drain inclusive of delimiter
                let mut frame = cobs_buf.drain(..=end_idx).collect::<Vec<u8>>();
                frame.pop(); // remove trailing 0x00 delimiter

                if frame.is_empty() {
                    continue;
                }

                // Decode into a new Vec (works across cobs versions)
                match cobs::decode_vec(&frame) {
                    Ok(mut decoded) => {
                        process_one(
                            start_time,
                            &mut decoded,
                            &mut total_accel_samples,
                            &mut total_combined_marker_samples,
                            addr,
                        );
                    }
                    Err(e) => {
                        eprintln!("COBS decode error: {e:?}; dropping frame");
                    }
                }
            }
        }
    }
}
