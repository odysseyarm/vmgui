use ats_usb::packet::Packet;
use ats_usb::packet::GeneralConfig;
use std::io::Read;
use std::io::Seek;
use std::io::Error;
use std::path::PathBuf;

pub fn read_file(path: &PathBuf) -> Result<(GeneralConfig, Vec<(i128, Packet)>), Error> {
    let mut file = std::fs::File::open(path).unwrap();
    let mut buf = [0; GeneralConfig::SIZE as usize];
    file.read_exact(&mut buf).unwrap();
    let general_config = GeneralConfig::parse(&mut &buf[..], ats_usb::packet::PacketType::ReadConfigResponse).unwrap();
    let mut packets = Vec::new();
    loop {
        let mut buf = [0; 16];
        match file.read_exact(&mut buf) {
            Ok(()) => {
                let timestamp = i128::from_le_bytes(buf);
                let mut chunk = Vec::new();
                let mut buf = [0; 1024];
                let read = file.read(&mut buf).unwrap();
                chunk.extend_from_slice(&buf[..read]);

                let chunk_slice = &mut &chunk[..];
                let prev_chunk_len = chunk_slice.len();
                match Packet::parse(chunk_slice) {
                    Ok(pkt) => { packets.push((timestamp, pkt)) },
                    Err(e) => {
                        if let ats_usb::packet::Error::UnexpectedEof { packet_type: _ } = e {
                            panic!("Unexpected EOF");
                        } else {
                            panic!("Error parsing packet: {:?}", e);
                        }
                    }
                };

                let new_chunk_len = chunk_slice.len();
                let seek_back_amount = read - (prev_chunk_len - new_chunk_len);
                file.seek(std::io::SeekFrom::Current(-(seek_back_amount as i64))).unwrap();
            } Err(e) => {
                if e.kind() == std::io::ErrorKind::UnexpectedEof {
                    break;
                } else {
                    panic!("Error reading file: {:?}", e);
                }
            }
        }
    }
    Ok((general_config, packets))
}
