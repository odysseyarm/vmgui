#![allow(unused)]
#[derive(Clone, Debug)]
pub enum Packet {
    WriteRegister(Register), // a.k.a. Poke
    ReadRegister(Register), // a.k.a. Peek
    ReadRegisterResponse(ReadRegisterResponse),
}

#[derive(Clone, Copy, Debug)]
pub struct Register {
    pub port: Port,
    pub bank: u8,
    pub address: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct ReadRegisterResponse {
    pub bank: u8,
    pub address: u8,
    pub data: u8,
}


#[derive(Clone, Copy, Debug)]
pub enum PacketParseError {
    UnexpectedEof,
    UnrecognizedPacketId,
    UnrecognizedPort,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum Port {
    Nv,
    Fv,
}
impl TryFrom<u8> for Port {
    type Error = PacketParseError;
    fn try_from(n: u8) -> Result<Self, Self::Error> {
        match n {
            0 => Ok(Self::Nv),
            1 => Ok(Self::Fv),
            _ => Err(PacketParseError::UnrecognizedPort),
        }
    }
}

#[repr(u8)]
pub enum PacketId {
    WriteRegister, // a.k.a. Poke
    ReadRegister,  // a.k.a. Peek
    ReadRegisterResponse,
}

impl TryFrom<u8> for PacketId {
    type Error = PacketParseError;
    fn try_from(n: u8) -> Result<Self, Self::Error> {
        match n {
            0 => Ok(Self::WriteRegister),
            1 => Ok(Self::ReadRegister),
            2 => Ok(Self::ReadRegisterResponse),
            _ => Err(PacketParseError::UnrecognizedPacketId),
        }
    }
}

impl Packet {

    pub fn id(&self) -> PacketId {
        use PacketId as P;
        match self {
            Self::WriteRegister(_) => P::WriteRegister,
            Self::ReadRegister(_) => P::ReadRegister,
            Self::ReadRegisterResponse(_) => P::ReadRegisterResponse,
        }
    }

    pub fn parse(bytes: &mut &[u8]) -> Result<Self, PacketParseError> {
        use PacketParseError as E;
        let [words1, words2, id, _, ..] = **bytes else {
            return Err(E::UnexpectedEof);
        };

        let words = u16::from_le_bytes([words1, words2]);
        let id = PacketId::try_from(id)?;

        let len = usize::from(words)*2;
        if bytes.len() < len {
            return Err(E::UnexpectedEof);
        }
        *bytes = &bytes[4..];
        Ok(match id {
            PacketId::WriteRegister => Self::WriteRegister(Register::parse(bytes)?),
            PacketId::ReadRegister => Self::ReadRegister(Register::parse(bytes)?),
            PacketId::ReadRegisterResponse => Self::ReadRegisterResponse(ReadRegisterResponse::parse(bytes)?),
        })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        // (u16 words/2, u8 tag, u8 padding)
        let len = match self {
            Packet::WriteRegister(_) => 4,
            Packet::ReadRegister(_) => 4,
            Packet::ReadRegisterResponse(_) => 4,
        };
        let words = u16::to_le_bytes((len + 4) / 2);
        let id = self.id();
        buf.reserve(4 + usize::from(len));
        buf.extend_from_slice(&[words[0], words[1], id as u8, 0]);
        match self {
            Packet::WriteRegister(x) => x.serialize(buf),
            Packet::ReadRegister(x) => x.serialize(buf),
            Packet::ReadRegisterResponse(x) => x.serialize(buf),
        };
    }
}

impl Register {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, PacketParseError> {
        use PacketParseError as E;
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
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, PacketParseError> {
        use PacketParseError as E;
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
