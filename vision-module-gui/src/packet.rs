#![allow(unused)]

use std::{fmt::Display, error::Error as StdError};
#[derive(Clone, Debug)]
pub struct Packet {
    pub data: PacketData,
    pub id: u8,
}
#[derive(Clone, Debug)]
pub enum PacketData {
    WriteRegister(WriteRegister), // a.k.a. Poke
    ReadRegister(Register), // a.k.a. Peek
    ReadRegisterResponse(ReadRegisterResponse),
    ObjectReportRequest(ObjectReportRequest),
    ObjectReport(ObjectReport),
    StreamUpdate(StreamUpdate),
    FlashSettings,
    AimPointReport,
    ImpactWithAimPointReport,
}
pub enum StreamChoice {
    Object,
    AimPoint,
    ImpactWithAimPoint,
}

#[derive(Clone, Copy, Debug)]
pub struct Register {
    pub port: Port,
    pub bank: u8,
    pub address: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct WriteRegister {
    pub port: Port,
    pub bank: u8,
    pub address: u8,
    pub data: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct ReadRegisterResponse {
    pub bank: u8,
    pub address: u8,
    pub data: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct ObjectReportRequest {}

#[derive(Clone, Copy, Debug, Default)]
pub struct MotData {
    pub area: u16,
    pub cx: u16,
    pub cy: u16,
    pub avg_brightness: u8,
    pub max_brightness: u8,
    pub range: u8,
    pub radius: u8,
    pub boundary_left: u8,
    pub boundary_right: u8,
    pub boundary_up: u8,
    pub boundary_down: u8,
    pub aspect_ratio: u8,
    pub vx: u8,
    pub vy: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct ObjectReport {
    pub mot_data_nf: [MotData; 16],
    pub mot_data_wf: [MotData; 16],
}

#[derive(Clone, Copy, Debug)]
pub struct StreamUpdate {
    pub mask: u8,
    pub active: bool,
}

#[derive(Clone, Copy, Debug)]
pub enum Error {
    UnexpectedEof,
    UnrecognizedPacketId,
    UnrecognizedPort,
}

impl Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        use Error as S;
        match self {
            S::UnexpectedEof => write!(f, "unexpected eof"),
            S::UnrecognizedPacketId => write!(f, "unrecognized packet id"),
            S::UnrecognizedPort => write!(f, "unrecognized port"),
        }
    }
}

impl StdError for Error {}

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum Port {
    Nf,
    Wf,
}
impl TryFrom<u8> for Port {
    type Error = Error;
    fn try_from(n: u8) -> Result<Self, Self::Error> {
        match n {
            0 => Ok(Self::Nf),
            1 => Ok(Self::Wf),
            _ => Err(Error::UnrecognizedPort),
        }
    }
}

#[repr(u8)]
pub enum PacketType {
    WriteRegister, // a.k.a. Poke
    ReadRegister,  // a.k.a. Peek
    ReadRegisterResponse,
    ObjectReportRequest,
    ObjectReport,
    StreamUpdate,
    FlashSettings,
    AimPointReport,
    ImpactWithAimPointReport,
    End,
}

impl TryFrom<u8> for PacketType {
    type Error = Error;
    fn try_from(n: u8) -> Result<Self, Self::Error> {
        match n {
            0 => Ok(Self::WriteRegister),
            1 => Ok(Self::ReadRegister),
            2 => Ok(Self::ReadRegisterResponse),
            3 => Ok(Self::ObjectReportRequest),
            4 => Ok(Self::ObjectReport),
            5 => Ok(Self::StreamUpdate),
            6 => Ok(Self::FlashSettings),
            7 => Ok(Self::AimPointReport),
            8 => Ok(Self::ImpactWithAimPointReport),
            9 => Ok(Self::End),
            _ => Err(Error::UnrecognizedPacketId),
        }
    }
}

impl Packet {

    pub fn ty(&self) -> PacketType {
        use PacketType as P;
        match self.data {
            PacketData::WriteRegister(_) => P::WriteRegister,
            PacketData::ReadRegister(_) => P::ReadRegister,
            PacketData::ReadRegisterResponse(_) => P::ReadRegisterResponse,
            PacketData::ObjectReportRequest(_) => P::ObjectReportRequest,
            PacketData::ObjectReport(_) => P::ObjectReport,
            PacketData::StreamUpdate(_) => P::StreamUpdate,
            PacketData::AimPointReport => P::AimPointReport,
            PacketData::ImpactWithAimPointReport => P::ImpactWithAimPointReport,
            PacketData::FlashSettings => P::FlashSettings,
        }
    }

    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        use Error as E;
        let [words1, words2, ty, id, ..] = **bytes else {
            return Err(E::UnexpectedEof);
        };

        let words = u16::from_le_bytes([words1, words2]);
        let ty = PacketType::try_from(ty)?;

        let len = usize::from(words)*2;
        if bytes.len() < len {
            return Err(E::UnexpectedEof);
        }
        *bytes = &bytes[4..];
        let data = match ty {
            PacketType::WriteRegister => PacketData::WriteRegister(WriteRegister::parse(bytes)?),
            PacketType::ReadRegister => PacketData::ReadRegister(Register::parse(bytes)?),
            PacketType::ReadRegisterResponse => PacketData::ReadRegisterResponse(ReadRegisterResponse::parse(bytes)?),
            PacketType::ObjectReportRequest => PacketData::ObjectReportRequest(ObjectReportRequest{}),
            PacketType::ObjectReport => PacketData::ObjectReport(ObjectReport::parse(bytes)?),
            // PacketType::StreamUpdate => PacketData::StreamUpdate(bytes[0] != 0),
            PacketType::FlashSettings => PacketData::FlashSettings,
            _ => unimplemented!(),
        };
        Ok(Self { id, data })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        // (u16 words/2, u8 tag, u8 id)
        let len = match self.data {
            PacketData::WriteRegister(_) => 4,
            PacketData::ReadRegister(_) => 4,
            PacketData::ReadRegisterResponse(_) => 4,
            PacketData::ObjectReportRequest(_) => 0,
            PacketData::ObjectReport(_) => 514,
            PacketData::StreamUpdate(_) => 2,
            PacketData::AimPointReport => 80,
            PacketData::ImpactWithAimPointReport => 80,
            PacketData::FlashSettings => 0,
        };
        let words = u16::to_le_bytes((len + 4) / 2);
        let ty = self.ty();
        buf.reserve(4 + usize::from(len));
        buf.extend_from_slice(&[words[0], words[1], ty as u8, self.id]);
        match &self.data {
            PacketData::WriteRegister(x) => x.serialize(buf),
            PacketData::ReadRegister(x) => x.serialize(buf),
            PacketData::ReadRegisterResponse(x) => x.serialize(buf),
            PacketData::ObjectReportRequest(_) => (),
            PacketData::ObjectReport(x) => x.serialize(buf),
            PacketData::StreamUpdate(x) => buf.extend_from_slice(&[x.mask as u8, x.active as u8]),
            PacketData::AimPointReport => (),
            PacketData::ImpactWithAimPointReport => (),
            PacketData::FlashSettings => (),
        };
    }
}

impl PacketData {
    pub fn read_register_response(self) -> Option<ReadRegisterResponse> {
        match self {
            PacketData::ReadRegisterResponse(x) => Some(x),
            _ => None,
        }
    }

    pub fn object_report(self) -> Option<ObjectReport> {
        match self {
            PacketData::ObjectReport(x) => Some(x),
            _ => None,
        }
    }
}

impl Register {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        use Error as E;
        let [port, bank, address, _, ..] = **bytes else {
            return Err(E::UnexpectedEof);
        };
        let port = port.try_into()?;
        *bytes = &bytes[4..];
        Ok(Self { port, bank, address })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&[self.port as u8, self.bank, self.address, 0]);
    }
}

impl ReadRegisterResponse {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        use Error as E;
        let [bank, address, data, _, ..] = **bytes else {
            return Err(E::UnexpectedEof);
        };
        *bytes = &bytes[4..];
        Ok(Self { bank, address, data })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&[self.bank, self.address, self.data, 0]);
    }
}

impl WriteRegister {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        use Error as E;
        let [port, bank, address, data, ..] = **bytes else {
            return Err(E::UnexpectedEof);
        };
        let port = port.try_into()?;
        *bytes = &bytes[4..];
        Ok(Self { port, bank, address, data })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&[self.port as u8, self.bank, self.address, self.data]);
    }
}

impl MotData {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        let mot_data = MotData {
            area: bytes[0] as u16 | ((bytes[1] as u16) << 8),
            cx: bytes[2] as u16 | ((bytes[3] & 0x0f) as u16) << 8,
            cy: bytes[4] as u16 | ((bytes[5] & 0x0f) as u16) << 8,
            avg_brightness: bytes[6],
            max_brightness: bytes[7],
            radius: bytes[8] & 0x0f,
            range: bytes[8] >> 4,
            boundary_left: bytes[9] & 0x7f,
            boundary_right: bytes[10] & 0x7f,
            boundary_up: bytes[11] & 0x7f,
            boundary_down: bytes[12] & 0x7f,
            aspect_ratio: bytes[13],
            vx: bytes[14],
            vy: bytes[15],
        };
        *bytes = &bytes[16..];
        Ok(mot_data)
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&[
            self.area as u8,
            (self.area >> 8) as u8,
            self.cx as u8,
            (self.cx >> 8) as u8,
            self.cy as u8,
            (self.cy >> 8) as u8,
            self.avg_brightness,
            self.max_brightness,
            self.radius | self.range << 4,
            self.boundary_left,
            self.boundary_right,
            self.boundary_up,
            self.boundary_down,
            self.aspect_ratio,
            self.vx,
            self.vy,
        ]);
    }
}

impl ObjectReport {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        use Error as E;
        let mut data = &mut &bytes[..512];
        *bytes = &bytes[512..];
        let [format, _, ..] = **bytes else {
            return Err(E::UnexpectedEof);
        };
        *bytes = &bytes[2..];
        Ok(Self { mot_data_nf: [(); 16].map(|_| MotData::parse(data).expect("MotData parse error")), mot_data_wf: [(); 16].map(|_| MotData::parse(data).expect("MotData parse error")) })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        for i in 0..16 {
            self.mot_data_nf[i].serialize(buf);
        }
        for i in 0..16 {
            self.mot_data_wf[i].serialize(buf);
        }
        buf.extend_from_slice(&[1, 0]);
    }
}
