use std::{fs, io, path::PathBuf};
use postcard::take_from_bytes;
use ats_usb::packets::vm::{GeneralConfig, Packet};

pub fn read_file(path: &PathBuf) -> io::Result<(GeneralConfig, Vec<(u128, Packet)>)> {
    let data = fs::read(path)?;
    let mut bytes: &[u8] = &data;

    let (general_config, rest) = take_from_bytes::<GeneralConfig>(bytes)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("GeneralConfig decode: {e:?}")))?;
    bytes = rest;

    let mut packets = Vec::new();
    while !bytes.is_empty() {
        if bytes.len() < 16 {
            return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "EOF in timestamp"));
        }

        let mut tsb = [0u8; 16];
        tsb.copy_from_slice(&bytes[..16]);
        let timestamp = u128::from_le_bytes(tsb);

        match take_from_bytes::<Packet>(&bytes[16..]) {
            Ok((pkt, rest_after_pkt)) => {
                let consumed = 16 + (bytes[16..].len() - rest_after_pkt.len());
                bytes = &bytes[consumed..];
                packets.push((timestamp, pkt));
            }
            Err(postcard::Error::DeserializeUnexpectedEnd) => {
                return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "EOF in Packet"));
            }
            Err(e) => {
                return Err(io::Error::new(io::ErrorKind::InvalidData, format!("Packet decode: {e:?}")));
            }
        }
    }

    Ok((general_config, packets))
}
