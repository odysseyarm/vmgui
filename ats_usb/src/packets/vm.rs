// Assume we're running on little-endian
use std::{error::Error as StdError, fmt::Display, mem::MaybeUninit};

use bytemuck::{AnyBitPattern, CheckedBitPattern, NoUninit};
use nalgebra::{Isometry3, Point2, Vector3};
use opencv_ros_camera::{Distortion, RosOpenCvIntrinsics};
use serde::Deserialize;
use serde_inline_default::serde_inline_default;

#[cfg(test)]
mod tests;
mod wire;

pub trait Parse: Sized {
    fn parse(bytes: &mut &[u8]) -> Result<Self, Error>;
}

pub trait Serialize {
    const SIZE: usize;
    #[must_use]
    fn serialize(&self, buf: &mut &mut [MaybeUninit<u8>]);
    fn serialize_to_vec(&self, buf: &mut Vec<u8>) {
        buf.reserve(Self::SIZE);
        let _ = self.serialize(&mut buf.spare_capacity_mut());
        unsafe {
            buf.set_len(buf.len() + Self::SIZE);
        }
    }
}

/// Implements [`Parse`] and [`Serialize`] via a "wire" type.
trait Delegated: From<Self::Wire> + Clone + Sized {
    type Wire: From<Self> + NoUninit + CheckedBitPattern;

    /// What value to put in the error when an `UnexpectedEof` error occurs.
    const PACKET_TYPE: Option<PacketType>;
}

impl<T: Delegated> Serialize for T {
    // make sure it's a multiple of 2
    const SIZE: usize = (std::mem::size_of::<T::Wire>() + 1) / 2 * 2;
    fn serialize<'a>(&self, buf: &mut &mut [MaybeUninit<u8>]) {
        let wire = T::Wire::from(self.clone());
        let wire_bytes = bytemuck::bytes_of(&wire);
        unsafe {
            let ptr = <[_]>::as_mut_ptr(buf) as *mut u8;
            ptr.copy_from_nonoverlapping(wire_bytes.as_ptr(), wire_bytes.len());
            ptr.add(wire_bytes.len()).write_bytes(0, T::SIZE - wire_bytes.len());
        }
        *buf = &mut std::mem::take(buf)[T::SIZE..];
    }
}

impl<T: Delegated> Parse for T {
    fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        let size_with_padding = T::SIZE;
        if bytes.len() < size_with_padding {
            Err(Error::UnexpectedEof { packet_type: Self::PACKET_TYPE })
        } else {
            let size = std::mem::size_of::<T::Wire>();
            match bytemuck::checked::try_pod_read_unaligned(&bytes[..size]) {
                Ok(v) => {
                    *bytes = &bytes[size_with_padding..];
                    Ok(Self::from(v))
                }
                Err(bytemuck::checked::CheckedCastError::InvalidBitPattern) => Err(Error::InvalidBitPattern),
                Err(e) => unreachable!("{e}"),
            }
        }
    }
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Debug)]
pub struct Packet {
    pub data: PacketData,
    pub id: u8,
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Debug)]
pub enum PacketData {
    WriteRegister(WriteRegister), // a.k.a. Poke
    ReadRegister(Register),       // a.k.a. Peek
    ReadRegisterResponse(ReadRegisterResponse),
    WriteConfig(GeneralConfig),
    ReadConfig(),
    ReadConfigResponse(GeneralConfig),
    ReadProps(),
    ReadPropsResponse(Props),
    ObjectReportRequest(),
    ObjectReport(ObjectReport),
    CombinedMarkersReport(CombinedMarkersReport),
    AccelReport(AccelReport),
    ImpactReport(ImpactReport),
    StreamUpdate(StreamUpdate),
    FlashSettings(),
    Vendor(u8, (u8, [u8; 98])),
}

#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[repr(u8)]
#[derive(Copy, Clone, Debug, enumn::N, PartialEq)]
pub enum StreamUpdateAction {
    Enable,
    Disable,
    DisableAll,
}

impl TryFrom<u8> for StreamUpdateAction {
    type Error = Error;
    fn try_from(n: u8) -> Result<Self, Self::Error> {
        Self::n(n).ok_or(Error::UnrecognizedStreamUpdateAction(n))
    }
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug, NoUninit, CheckedBitPattern)]
pub struct Register {
    pub port: Port,
    pub bank: u8,
    pub address: u8,
}

impl Delegated for Register {
    type Wire = Self;
    const PACKET_TYPE: Option<PacketType> = Some(PacketType::ReadRegister());
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug, NoUninit, CheckedBitPattern)]
pub struct WriteRegister {
    pub port: Port,
    pub bank: u8,
    pub address: u8,
    pub data: u8,
}

impl Delegated for WriteRegister {
    type Wire = Self;
    const PACKET_TYPE: Option<PacketType> = Some(PacketType::WriteRegister());
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug, NoUninit, AnyBitPattern)]
pub struct ReadRegisterResponse {
    pub bank: u8,
    pub address: u8,
    pub data: u8,
}

impl Delegated for ReadRegisterResponse {
    type Wire = Self;
    const PACKET_TYPE: Option<PacketType> = Some(PacketType::ReadRegisterResponse());
}

#[repr(C)]
#[serde_inline_default]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(serde::Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct AccelConfig {
    #[serde_inline_default(100)]
    pub accel_odr: u16,
    pub b_x: f32,
    pub b_y: f32,
    pub b_z: f32,
    pub s_x: f32,
    pub s_y: f32,
    pub s_z: f32,
}

impl Default for AccelConfig {
    fn default() -> Self {
        Self {
            accel_odr: 100,
            b_x: 0.0,
            b_y: 0.0,
            b_z: 0.0,
            s_x: 1.0,
            s_y: 1.0,
            s_z: 1.0,
        }
    }
}

#[repr(C)]
#[serde_inline_default]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(serde::Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq, NoUninit, AnyBitPattern)]
pub struct GyroConfig {
    pub b_x: f32,
    pub b_y: f32,
    pub b_z: f32,
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Clone, Debug, PartialEq)]
pub struct GeneralConfig {
    pub impact_threshold: u8,
    pub accel_config: AccelConfig,
    pub gyro_config: GyroConfig,
    pub camera_model_nf: RosOpenCvIntrinsics<f32>,
    pub camera_model_wf: RosOpenCvIntrinsics<f32>,
    pub stereo_iso: Isometry3<f32>,
}

impl Delegated for GeneralConfig {
    type Wire = wire::GeneralConfig;
    const PACKET_TYPE: Option<PacketType> = None;
}

impl Default for GeneralConfig {
    fn default() -> Self {
        Self {
            impact_threshold: 5,
            accel_config: Default::default(),
            gyro_config: Default::default(),
            camera_model_nf: RosOpenCvIntrinsics::from_params(
                5896.181, 0., 5896.181, 2047.5, 2047.5,
            ),
            camera_model_wf: RosOpenCvIntrinsics::from_params_with_distortion(
                1434.9723,
                0.,
                1437.214,
                2036.575,
                2088.8027,
                Distortion::from_opencv_vec(
                    [
                        0.039820533,
                        -0.03993317,
                        0.00043006078,
                        -0.0012057066,
                        0.005302235,
                    ]
                    .into(),
                ),
            ),
            stereo_iso: Isometry3::identity(),
        }
    }
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Clone, Copy, Debug, Default, NoUninit, AnyBitPattern)]
pub struct Props {
    pub uuid: [u8; 6],
}

impl Delegated for Props {
    type Wire = Self;
    const PACKET_TYPE: Option<PacketType> = None;
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
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

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ObjectReport {
    pub timestamp: u32,
    pub mot_data_nf: [MotData; 16],
    pub mot_data_wf: [MotData; 16],
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CombinedMarkersReport {
    pub nf_points: [Point2<u16>; 16],
    pub wf_points: [Point2<u16>; 16],
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AccelReport {
    pub timestamp: u32,
    pub accel: Vector3<f32>,
    pub gyro: Vector3<f32>,
}

impl Delegated for AccelReport {
    type Wire = wire::AccelReport;
    const PACKET_TYPE: Option<PacketType> = Some(PacketType::AccelReport());
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug, Default, NoUninit, AnyBitPattern)]
pub struct ImpactReport {
    pub timestamp: u32,
}

impl Delegated for ImpactReport {
    type Wire = Self;
    const PACKET_TYPE: Option<PacketType> = Some(PacketType::ImpactReport());
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug)]
pub struct StreamUpdate {
    pub packet_id: PacketType,
    pub action: StreamUpdateAction,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub enum Error {
    UnexpectedEof { packet_type: Option<PacketType> },
    UnrecognizedPacketId(u8),
    UnrecognizedPort,
    UnrecognizedStreamUpdateAction(u8),
    InvalidBitPattern,
}

impl Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        use Error as S;
        match self {
            S::UnexpectedEof { packet_type: None } => write!(f, "unexpected eof"),
            S::UnexpectedEof {
                packet_type: Some(p),
            } => write!(f, "unexpected eof, packet id {p:?}"),
            S::UnrecognizedPacketId(id) => write!(f, "unrecognized packet id {id}"),
            S::UnrecognizedPort => write!(f, "unrecognized port"),
            S::UnrecognizedStreamUpdateAction(n) => {
                write!(f, "unrecognized stream update action {n}")
            }
            S::InvalidBitPattern => write!(f, "invalid bit pattern"),
        }
    }
}

impl StdError for Error {}

#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[repr(u8)]
#[derive(Clone, Copy, Debug, NoUninit, CheckedBitPattern)]
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

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Copy, Clone, Debug)]
pub enum PacketType {
    WriteRegister(), // a.k.a. Poke
    ReadRegister(),  // a.k.a. Peek
    ReadRegisterResponse(),
    WriteConfig(),
    ReadConfig(),
    ReadConfigResponse(),
    ReadProps(),
    ReadPropsResponse(),
    ObjectReportRequest(),
    ObjectReport(),
    CombinedMarkersReport(),
    AccelReport(),
    ImpactReport(),
    StreamUpdate(),
    FlashSettings(),
    End(),
    VendorStart(),
    Vendor(u8),
    VendorEnd(),
}

impl TryFrom<u8> for PacketType {
    type Error = Error;
    fn try_from(n: u8) -> Result<Self, Self::Error> {
        match n {
            0x00 => Ok(Self::WriteRegister()),
            0x01 => Ok(Self::ReadRegister()),
            0x02 => Ok(Self::ReadRegisterResponse()),
            0x03 => Ok(Self::WriteConfig()),
            0x04 => Ok(Self::ReadConfig()),
            0x05 => Ok(Self::ReadConfigResponse()),
            0x06 => Ok(Self::ReadProps()),
            0x07 => Ok(Self::ReadPropsResponse()),
            0x08 => Ok(Self::ObjectReportRequest()),
            0x09 => Ok(Self::ObjectReport()),
            0x0a => Ok(Self::CombinedMarkersReport()),
            0x0b => Ok(Self::AccelReport()),
            0x0c => Ok(Self::ImpactReport()),
            0x0d => Ok(Self::StreamUpdate()),
            0x0e => Ok(Self::FlashSettings()),
            0x0f => Ok(Self::End()),
            0x80 => Ok(Self::VendorStart()),
            0xff => Ok(Self::VendorEnd()),
            n if (PacketType::VendorStart().into()..PacketType::VendorEnd().into())
                .contains(&n) =>
            {
                Ok(Self::Vendor(n))
            }
            _ => Err(Error::UnrecognizedPacketId(n)),
        }
    }
}

impl From<PacketType> for u8 {
    fn from(ty: PacketType) -> u8 {
        match ty {
            PacketType::WriteRegister() => 0x00,
            PacketType::ReadRegister() => 0x01,
            PacketType::ReadRegisterResponse() => 0x02,
            PacketType::WriteConfig() => 0x03,
            PacketType::ReadConfig() => 0x04,
            PacketType::ReadConfigResponse() => 0x05,
            PacketType::ReadProps() => 0x06,
            PacketType::ReadPropsResponse() => 0x07,
            PacketType::ObjectReportRequest() => 0x08,
            PacketType::ObjectReport() => 0x09,
            PacketType::CombinedMarkersReport() => 0x0a,
            PacketType::AccelReport() => 0x0b,
            PacketType::ImpactReport() => 0x0c,
            PacketType::StreamUpdate() => 0x0d,
            PacketType::FlashSettings() => 0x0e,
            PacketType::End() => 0x0f,
            PacketType::VendorStart() => 0x80,
            PacketType::VendorEnd() => 0xff,
            PacketType::Vendor(n) => n,
        }
    }
}

impl Packet {
    pub fn ty(&self) -> PacketType {
        match self.data {
            PacketData::WriteRegister(_) => PacketType::WriteRegister(),
            PacketData::ReadRegister(_) => PacketType::ReadRegister(),
            PacketData::ReadRegisterResponse(_) => PacketType::ReadRegisterResponse(),
            PacketData::WriteConfig(_) => PacketType::WriteConfig(),
            PacketData::ReadConfig() => PacketType::ReadConfig(),
            PacketData::ReadConfigResponse(_) => PacketType::ReadConfigResponse(),
            PacketData::ReadProps() => PacketType::ReadProps(),
            PacketData::ReadPropsResponse(_) => PacketType::ReadPropsResponse(),
            PacketData::ObjectReportRequest() => PacketType::ObjectReportRequest(),
            PacketData::ObjectReport(_) => PacketType::ObjectReport(),
            PacketData::CombinedMarkersReport(_) => PacketType::CombinedMarkersReport(),
            PacketData::AccelReport(_) => PacketType::AccelReport(),
            PacketData::ImpactReport(_) => PacketType::ImpactReport(),
            PacketData::StreamUpdate(_) => PacketType::StreamUpdate(),
            PacketData::FlashSettings() => PacketType::FlashSettings(),
            PacketData::Vendor(n, _) => PacketType::Vendor(n),
        }
    }

    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        let [words1, words2, ty, id, ..] = **bytes else {
            return Err(Error::UnexpectedEof { packet_type: None });
        };

        let words = u16::from_le_bytes([words1, words2]);
        let ty = PacketType::try_from(ty)?;

        let len = usize::from(words) * 2;
        if bytes.len() < len {
            return Err(Error::UnexpectedEof {
                packet_type: Some(ty),
            });
        }
        *bytes = &bytes[4..];
        let data = match ty {
            PacketType::WriteRegister() => PacketData::WriteRegister(WriteRegister::parse(bytes)?),
            PacketType::ReadRegister() => PacketData::ReadRegister(Register::parse(bytes)?),
            PacketType::ReadRegisterResponse() => {
                PacketData::ReadRegisterResponse(ReadRegisterResponse::parse(bytes)?)
            }
            PacketType::WriteConfig() => unimplemented!(),
            PacketType::ReadConfig() => PacketData::ReadConfig(),
            PacketType::ReadConfigResponse() => {
                PacketData::ReadConfigResponse(GeneralConfig::parse(bytes)?)
            }
            PacketType::ReadProps() => PacketData::ReadProps(),
            PacketType::ReadPropsResponse() => {
                PacketData::ReadPropsResponse(Props::parse(bytes)?)
            }
            PacketType::ObjectReportRequest() => PacketData::ObjectReportRequest(),
            PacketType::ObjectReport() => PacketData::ObjectReport(ObjectReport::parse(bytes)?),
            PacketType::CombinedMarkersReport() => {
                PacketData::CombinedMarkersReport(CombinedMarkersReport::parse(bytes)?)
            }
            PacketType::AccelReport() => PacketData::AccelReport(AccelReport::parse(bytes)?),
            PacketType::ImpactReport() => PacketData::ImpactReport(ImpactReport::parse(bytes)?),
            PacketType::StreamUpdate() => PacketData::StreamUpdate(StreamUpdate::parse(bytes)?),
            PacketType::FlashSettings() => PacketData::FlashSettings(),
            PacketType::Vendor(n) => {
                let len = bytes[0];
                let mut data = [0; 98];
                data[..len as usize].copy_from_slice(&bytes[1..len as usize + 1]);
                *bytes = &bytes[len as usize + 1..];
                PacketData::Vendor(n, (len, data))
            }
            p => unimplemented!("{:?}", p),
        };
        Ok(Self { id, data })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        let len = match &self.data {
            PacketData::WriteRegister(_) => WriteRegister::SIZE as u16,
            PacketData::ReadRegister(_) => Register::SIZE as u16,
            PacketData::ReadRegisterResponse(_) => ReadRegisterResponse::SIZE as u16,
            PacketData::WriteConfig(_) => GeneralConfig::SIZE as u16,
            PacketData::ReadConfig() => 0,
            PacketData::ReadConfigResponse(_) => GeneralConfig::SIZE as u16,
            PacketData::ReadProps() => 0,
            PacketData::ReadPropsResponse(_) => 6,
            PacketData::ObjectReportRequest() => 0,
            PacketData::ObjectReport(_) => ObjectReport::SIZE,
            PacketData::CombinedMarkersReport(_) => CombinedMarkersReport::SIZE,
            PacketData::AccelReport(_) => AccelReport::SIZE as u16,
            PacketData::ImpactReport(_) => 4,
            PacketData::StreamUpdate(_) => 2,
            PacketData::FlashSettings() => 0,
            PacketData::Vendor(_, (len, _)) => {
                if *len % 2 != 0 {
                    (*len + 1) as u16
                } else {
                    *len as u16 + 2
                }
            }
        };
        let words = u16::to_le_bytes((len + 4) / 2);
        let ty = self.ty();
        buf.reserve(4 + usize::from(len));
        buf.extend_from_slice(&[words[0], words[1], ty.into(), self.id]);
        match &self.data {
            PacketData::WriteRegister(x) => x.serialize_to_vec(buf),
            PacketData::ReadRegister(x) => x.serialize_to_vec(buf),
            PacketData::ReadRegisterResponse(x) => x.serialize_to_vec(buf),
            PacketData::WriteConfig(x) => x.serialize_to_vec(buf),
            PacketData::ReadConfig() => (),
            PacketData::ReadConfigResponse(x) => x.serialize_to_vec(buf),
            PacketData::ReadProps() => (),
            PacketData::ReadPropsResponse(x) => x.serialize_to_vec(buf),
            PacketData::ObjectReportRequest() => (),
            PacketData::ObjectReport(x) => x.serialize(buf),
            PacketData::CombinedMarkersReport(x) => x.serialize(buf),
            PacketData::AccelReport(x) => x.serialize_to_vec(buf),
            PacketData::ImpactReport(x) => x.serialize_to_vec(buf),
            PacketData::StreamUpdate(x) => {
                buf.extend_from_slice(&[x.packet_id.into(), x.action as u8])
            }
            PacketData::FlashSettings() => (),
            PacketData::Vendor(_, (len, data)) => {
                buf.push(*len);
                buf.extend_from_slice(&data[..*len as usize]);
                if len % 2 == 0 {
                    buf.push(0);
                }
            }
        }
    }
}

impl PacketData {
    pub fn read_register_response(self) -> Option<ReadRegisterResponse> {
        match self {
            PacketData::ReadRegisterResponse(x) => Some(x),
            _ => None,
        }
    }

    pub fn read_config_response(self) -> Option<GeneralConfig> {
        match self {
            PacketData::ReadConfigResponse(x) => Some(x),
            _ => None,
        }
    }

    pub fn read_props_response(self) -> Option<Props> {
        match self {
            PacketData::ReadPropsResponse(x) => Some(x),
            _ => None,
        }
    }

    pub fn object_report(self) -> Option<ObjectReport> {
        match self {
            PacketData::ObjectReport(x) => Some(x),
            _ => None,
        }
    }

    pub fn combined_markers_report(self) -> Option<CombinedMarkersReport> {
        match self {
            PacketData::CombinedMarkersReport(x) => Some(x),
            _ => None,
        }
    }

    pub fn accel_report(self) -> Option<AccelReport> {
        match self {
            PacketData::AccelReport(x) => Some(x),
            _ => None,
        }
    }

    pub fn impact_report(self) -> Option<ImpactReport> {
        match self {
            PacketData::ImpactReport(x) => Some(x),
            _ => None,
        }
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
    const SIZE: u16 = 518;
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        use Error as E;
        let timestamp = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        *bytes = &bytes[4..];
        let data = &mut &bytes[..512];
        *bytes = &bytes[512..];
        let [_format, _, ..] = **bytes else {
            return Err(E::UnexpectedEof {
                packet_type: Some(PacketType::ObjectReport()),
            });
        };
        *bytes = &bytes[2..];
        Ok(Self {
            timestamp,
            mot_data_nf: [(); 16].map(|_| MotData::parse(data).expect("MotData parse error")),
            mot_data_wf: [(); 16].map(|_| MotData::parse(data).expect("MotData parse error")),
        })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&self.timestamp.to_le_bytes());
        for i in 0..16 {
            self.mot_data_nf[i].serialize(buf);
        }
        for i in 0..16 {
            self.mot_data_wf[i].serialize(buf);
        }
        buf.extend_from_slice(&[1, 0]);
    }
}

impl CombinedMarkersReport {
    const SIZE: u16 = 96;
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        use Error as E;
        let size = Self::SIZE as usize;
        if bytes.len() < size {
            return Err(E::UnexpectedEof {
                packet_type: Some(PacketType::CombinedMarkersReport()),
            });
        }

        let data = &mut &bytes[..size];
        *bytes = &bytes[size..];

        let mut positions = [Point2::new(0, 0); 16 * 2];
        for i in 0..positions.len() {
            // x, y is 12 bits each
            let x = u16::from_le_bytes([data[0], data[1] & 0x0f]);
            let y = (data[1] >> 4) as u16 | ((data[2] as u16) << 4);
            positions[i] = Point2::new(x, y);
            *data = &data[3..];
        }
        let nf_positions = positions[..16].try_into().unwrap();
        let wf_positions = positions[16..].try_into().unwrap();

        Ok(Self {
            nf_points: nf_positions,
            wf_points: wf_positions,
        })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        for p in self.nf_points.iter().chain(&self.wf_points) {
            let (x, y) = (p.x, p.y);
            let byte0 = x & 0xff;
            let byte1 = ((x >> 8) & 0x0f) | ((y & 0x0f) << 4);
            let byte2 = y >> 4;
            buf.extend_from_slice(&[byte0 as u8, byte1 as u8, byte2 as u8]);
        }
    }
}

#[cfg(feature = "pyo3")]
#[pyo3::pymethods]
impl CombinedMarkersReport {
    #[getter]
    fn nf_points(&self) -> [[u16; 2]; 16] {
        self.nf_points.map(|p| [p.x, p.y])
    }
    #[getter]
    fn wf_points(&self) -> [[u16; 2]; 16] {
        self.wf_points.map(|p| [p.x, p.y])
    }
}

impl AccelReport {
    pub fn corrected_accel(&self, accel_config: &AccelConfig) -> Vector3<f32> {
        Vector3::new(
            (self.accel.x - accel_config.b_x) / accel_config.s_x,
            (self.accel.y - accel_config.b_y) / accel_config.s_y,
            (self.accel.z - accel_config.b_z) / accel_config.s_z,
        )
    }

    pub fn corrected_gyro(&self, gyro_config: &GyroConfig) -> Vector3<f32> {
        Vector3::new(
            self.gyro.x - gyro_config.b_x,
            self.gyro.y - gyro_config.b_y,
            self.gyro.z - gyro_config.b_z,
        )
    }
}

#[cfg(feature = "pyo3")]
#[pyo3::pymethods]
impl AccelReport {
    #[getter]
    fn timestamp(&self) -> u32 {
        self.timestamp
    }
}

impl StreamUpdate {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        let stream_update = StreamUpdate {
            packet_id: bytes[0].try_into()?,
            action: bytes[1].try_into()?,
        };
        *bytes = &bytes[2..];
        Ok(stream_update)
    }
}
