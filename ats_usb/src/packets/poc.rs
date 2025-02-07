use std::{error::Error as StdError, fmt::Display};

use ats_common::ocv_types::{MinimalCameraCalibrationParams, MinimalStereoCalibrationParams};
use nalgebra::{Isometry3, Point2, Vector3};
use opencv_ros_camera::{Distortion, RosOpenCvIntrinsics};
use serde::{Deserialize, Serialize};
use serde_inline_default::serde_inline_default;

#[cfg(test)]
mod tests;

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
    WriteRegisterResponse(),
    ReadRegister(Register),       // a.k.a. Peek
    ReadRegisterResponse(ReadRegisterResponse),
    WriteConfig(GeneralConfig),
    WriteConfigResponse(),
    ReadConfig(),
    ReadConfigResponse(GeneralConfig),
    ReadProps(),
    ReadPropsResponse(Props),
    ObjectReportRequest(ObjectReportRequest),
    ObjectReport(ObjectReport),
    MarkersReport(MarkersReport),
    AccelReport(AccelReport),
    ImpactReport(ImpactReport),
    StreamUpdate(StreamUpdate),
    FlashSettings(),
    FlashSettingsResponse(),
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
#[derive(Clone, Copy, Debug)]
pub struct Register {
    pub bank: u8,
    pub address: u8,
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug)]
pub struct WriteRegister {
    pub bank: u8,
    pub address: u8,
    pub data: u8,
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug)]
pub struct ReadRegisterResponse {
    pub bank: u8,
    pub address: u8,
    pub data: u8,
}

#[repr(C)]
#[serde_inline_default]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
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

impl AccelConfig {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        let accel_odr = u16::from_le_bytes([bytes[0], bytes[1]]);
        let b_x = f32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]);
        let b_y = f32::from_le_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]);
        let b_z = f32::from_le_bytes([bytes[10], bytes[11], bytes[12], bytes[13]]);
        let s_x = f32::from_le_bytes([bytes[14], bytes[15], bytes[16], bytes[17]]);
        let s_y = f32::from_le_bytes([bytes[18], bytes[19], bytes[20], bytes[21]]);
        let s_z = f32::from_le_bytes([bytes[22], bytes[23], bytes[24], bytes[25]]);
        *bytes = &bytes[26..];
        Ok(Self {
            accel_odr,
            b_x,
            b_y,
            b_z,
            s_x,
            s_y,
            s_z,
        })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&self.accel_odr.to_le_bytes());
        for &b in &[self.b_x, self.b_y, self.b_z, self.s_x, self.s_y, self.s_z] {
            buf.extend_from_slice(&b.to_le_bytes());
        }
    }
}

#[repr(C)]
#[serde_inline_default]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct GyroConfig {
    pub b_x: f32,
    pub b_y: f32,
    pub b_z: f32,
}

impl Default for GyroConfig {
    fn default() -> Self {
        Self {
            b_x: 0.0,
            b_y: 0.0,
            b_z: 0.0,
        }
    }
}

impl GyroConfig {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        let b_x = f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let b_y = f32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        let b_z = f32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
        *bytes = &bytes[12..];
        Ok(Self { b_x, b_y, b_z })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        for &b in &[self.b_x, self.b_y, self.b_z] {
            buf.extend_from_slice(&b.to_le_bytes());
        }
    }
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

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Clone, Debug)]
pub struct Props {
    pub uuid: [u8; 6],
}

impl Default for Props {
    fn default() -> Self {
        Self { uuid: [0; 6] }
    }
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
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug)]
pub struct ObjectReportRequest {}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotData {
    pub id: u8,
    pub avg_brightness: u8,
    pub area: u16,
    pub x: f32,
    pub y: f32,
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ObjectReport {
    pub timestamp: u32,
    pub mot_data: [MotData; 16],
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MarkersReport {
    pub points: [Point2<f32>; 16],
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AccelReport {
    pub timestamp: u32,
    pub accel: Vector3<f32>,
    pub gyro: Vector3<f32>,
}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug, Default)]
pub struct ImpactReport {
    pub timestamp: u32,
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
    UnrecognizedStreamUpdateAction(u8),
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
            S::UnrecognizedStreamUpdateAction(n) => {
                write!(f, "unrecognized stream update action {n}")
            }
        }
    }
}

impl StdError for Error {}

#[repr(C)]
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Copy, Clone, Debug)]
pub enum PacketType {
    WriteRegister(), // a.k.a. Poke
    WriteRegisterResponse(),
    ReadRegister(),  // a.k.a. Peek
    ReadRegisterResponse(),
    WriteConfig(),
    WriteConfigResponse(),
    ReadConfig(),
    ReadConfigResponse(),
    ReadProps(),
    ReadPropsResponse(),
    ObjectReportRequest(),
    ObjectReport(),
    MarkersReport(),
    AccelReport(),
    ImpactReport(),
    StreamUpdate(),
    FlashSettings(),
    FlashSettingsResponse(),
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
            0x0a => Ok(Self::MarkersReport()),
            0x0b => Ok(Self::AccelReport()),
            0x0c => Ok(Self::ImpactReport()),
            0x0d => Ok(Self::StreamUpdate()),
            0x0e => Ok(Self::FlashSettings()),
            0x0f => Ok(Self::FlashSettingsResponse()),
            0x10 => Ok(Self::End()),
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
            PacketType::WriteRegisterResponse() => 0x01,
            PacketType::ReadRegister() => 0x02,
            PacketType::ReadRegisterResponse() => 0x03,
            PacketType::WriteConfig() => 0x04,
            PacketType::WriteConfigResponse() => 0x05,
            PacketType::ReadConfig() => 0x06,
            PacketType::ReadConfigResponse() => 0x07,
            PacketType::ReadProps() => 0x08,
            PacketType::ReadPropsResponse() => 0x09,
            PacketType::ObjectReportRequest() => 0x0a,
            PacketType::ObjectReport() => 0x0b,
            PacketType::MarkersReport() => 0x0c,
            PacketType::AccelReport() => 0x0d,
            PacketType::ImpactReport() => 0x0e,
            PacketType::StreamUpdate() => 0x0f,
            PacketType::FlashSettings() => 0x10,
            PacketType::FlashSettingsResponse() => 0x11,
            PacketType::End() => 0x12,
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
            PacketData::WriteRegisterResponse() => PacketType::WriteRegisterResponse(),
            PacketData::ReadRegister(_) => PacketType::ReadRegister(),
            PacketData::ReadRegisterResponse(_) => PacketType::ReadRegisterResponse(),
            PacketData::WriteConfig(_) => PacketType::WriteConfig(),
            PacketData::WriteConfigResponse() => PacketType::WriteConfigResponse(),
            PacketData::ReadConfig() => PacketType::ReadConfig(),
            PacketData::ReadConfigResponse(_) => PacketType::ReadConfigResponse(),
            PacketData::ReadProps() => PacketType::ReadProps(),
            PacketData::ReadPropsResponse(_) => PacketType::ReadPropsResponse(),
            PacketData::ObjectReportRequest(_) => PacketType::ObjectReportRequest(),
            PacketData::ObjectReport(_) => PacketType::ObjectReport(),
            PacketData::MarkersReport(_) => PacketType::MarkersReport(),
            PacketData::AccelReport(_) => PacketType::AccelReport(),
            PacketData::ImpactReport(_) => PacketType::ImpactReport(),
            PacketData::StreamUpdate(_) => PacketType::StreamUpdate(),
            PacketData::FlashSettings() => PacketType::FlashSettings(),
            PacketData::FlashSettingsResponse() => PacketType::FlashSettingsResponse(),
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
            PacketType::WriteRegisterResponse() => PacketData::WriteRegisterResponse(),
            PacketType::ReadRegister() => PacketData::ReadRegister(Register::parse(bytes, ty)?),
            PacketType::ReadRegisterResponse() => {
                PacketData::ReadRegisterResponse(ReadRegisterResponse::parse(bytes)?)
            }
            PacketType::WriteConfig() => unimplemented!(),
            PacketType::WriteConfigResponse() => PacketData::WriteConfigResponse(),
            PacketType::ReadConfig() => PacketData::ReadConfig(),
            PacketType::ReadConfigResponse() => {
                PacketData::ReadConfigResponse(GeneralConfig::parse(bytes, ty)?)
            }
            PacketType::ReadProps() => PacketData::ReadProps(),
            PacketType::ReadPropsResponse() => {
                PacketData::ReadPropsResponse(Props::parse(bytes, ty)?)
            }
            PacketType::ObjectReportRequest() => {
                PacketData::ObjectReportRequest(ObjectReportRequest {})
            }
            PacketType::ObjectReport() => PacketData::ObjectReport(ObjectReport::parse(bytes)?),
            PacketType::MarkersReport() => {
                PacketData::MarkersReport(MarkersReport::parse(bytes)?)
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
        macro_rules! calculate_length {
            ($ty:ty) => {{
                assert_eq!(std::mem::align_of::<$ty>(), 1);
                std::mem::size_of::<$ty>() as u16
            }};
        }

        let len = match &self.data {
            PacketData::WriteRegister(_) => calculate_length!(WriteRegister),
            PacketData::WriteRegisterResponse() => 0,
            PacketData::ReadRegister(_) => calculate_length!(Register) + 1,
            PacketData::ReadRegisterResponse(_) => calculate_length!(ReadRegisterResponse) + 1,
            PacketData::WriteConfig(_) => GeneralConfig::SIZE,
            PacketData::WriteConfigResponse() => 0,
            PacketData::ReadConfig() => 0,
            PacketData::ReadConfigResponse(_) => GeneralConfig::SIZE,
            PacketData::ReadProps() => 0,
            PacketData::ReadPropsResponse(_) => 6,
            PacketData::ObjectReportRequest(_) => calculate_length!(ObjectReportRequest),
            PacketData::ObjectReport(_) => ObjectReport::SIZE,
            PacketData::MarkersReport(_) => MarkersReport::SIZE,
            PacketData::AccelReport(_) => 16,
            PacketData::ImpactReport(_) => 4,
            PacketData::StreamUpdate(_) => 2,
            PacketData::FlashSettings() => 0,
            PacketData::FlashSettingsResponse() => 0,
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
            PacketData::WriteRegister(x) => x.serialize(buf),
            PacketData::WriteRegisterResponse() => (),
            PacketData::ReadRegister(x) => x.serialize(buf),
            PacketData::ReadRegisterResponse(x) => x.serialize(buf),
            PacketData::WriteConfig(x) => x.serialize(buf),
            PacketData::WriteConfigResponse() => (),
            PacketData::ReadConfig() => (),
            PacketData::ReadConfigResponse(x) => x.serialize(buf),
            PacketData::ReadProps() => (),
            PacketData::ReadPropsResponse(x) => x.serialize(buf),
            PacketData::ObjectReportRequest(_) => (),
            PacketData::ObjectReport(x) => x.serialize(buf),
            PacketData::MarkersReport(x) => x.serialize(buf),
            PacketData::AccelReport(x) => x.serialize(buf),
            PacketData::ImpactReport(x) => x.serialize(buf),
            PacketData::StreamUpdate(x) => {
                buf.extend_from_slice(&[x.packet_id.into(), x.action as u8])
            }
            PacketData::FlashSettings() => (),
            PacketData::FlashSettingsResponse() => (),
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

    pub fn markers_report(self) -> Option<MarkersReport> {
        match self {
            PacketData::MarkersReport(x) => Some(x),
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

impl Register {
    pub fn parse(bytes: &mut &[u8], pkt_ty: PacketType) -> Result<Self, Error> {
        use Error as E;
        let [bank, address, _, ..] = **bytes else {
            return Err(E::UnexpectedEof {
                packet_type: Some(pkt_ty),
            });
        };
        *bytes = &bytes[2..];
        Ok(Self {
            bank,
            address,
        })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&[self.bank, self.address]);
    }
}

impl ReadRegisterResponse {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        use Error as E;
        let [bank, address, data, _, ..] = **bytes else {
            return Err(E::UnexpectedEof {
                packet_type: Some(PacketType::ReadRegisterResponse()),
            });
        };
        *bytes = &bytes[4..];
        Ok(Self {
            bank,
            address,
            data,
        })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&[self.bank, self.address, self.data, 0]);
    }
}

impl WriteRegister {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        use Error as E;
        let [bank, address, data, ..] = **bytes else {
            return Err(E::UnexpectedEof {
                packet_type: Some(PacketType::WriteRegister()),
            });
        };
        *bytes = &bytes[2..];
        Ok(Self {
            bank,
            address,
            data,
        })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&[self.bank, self.address, self.data]);
    }
}

impl GeneralConfig {
    pub const SIZE: u16 = 200;
    pub fn parse(bytes: &mut &[u8], _pkt_ty: PacketType) -> Result<Self, Error> {
        use Error as E;
        let impact_threshold = bytes[0];

        *bytes = &bytes[1..];

        let accel_config = match AccelConfig::parse(bytes) {
            Ok(x) => x,
            Err(_) => return Err(E::UnexpectedEof { packet_type: None }),
        };

        let gyro_config = match GyroConfig::parse(bytes) {
            Ok(x) => x,
            Err(_) => return Err(E::UnexpectedEof { packet_type: None }),
        };

        let mut camera_model_nf: RosOpenCvIntrinsics<f32> =
            match ats_common::ocv_types::MinimalCameraCalibrationParams::parse(bytes) {
                Ok(x) => x.into(),
                Err(_) => return Err(E::UnexpectedEof { packet_type: None }),
            };

        let mut camera_model_wf: RosOpenCvIntrinsics<f32> =
            match ats_common::ocv_types::MinimalCameraCalibrationParams::parse(bytes) {
                Ok(x) => x.into(),
                Err(_) => return Err(E::UnexpectedEof { packet_type: None }),
            };

        let stereo_iso = match ats_common::ocv_types::MinimalStereoCalibrationParams::parse(bytes) {
            Ok(x) => x.into(),
            Err(_) => return Err(E::UnexpectedEof { packet_type: None }),
        };

        *bytes = &bytes[1..];

        // HACK old configs use camera calibration based on 98x98.
        // Check cx and rescale to 4095x4095
        if camera_model_nf.p.m13 < 100.0 {
            eprintln!("please recalibrate nearfield");
            camera_model_nf.p.m11 *= 4095.0 / 98.0;
            camera_model_nf.p.m22 *= 4095.0 / 98.0;
            camera_model_nf.p.m13 *= 4095.0 / 98.0;
            camera_model_nf.p.m23 *= 4095.0 / 98.0;
        }
        if camera_model_wf.p.m13 < 100.0 {
            eprintln!("please recalibrate widefield");
            camera_model_wf.p.m11 *= 4095.0 / 98.0;
            camera_model_wf.p.m22 *= 4095.0 / 98.0;
            camera_model_wf.p.m13 *= 4095.0 / 98.0;
            camera_model_wf.p.m23 *= 4095.0 / 98.0;
        }

        Ok(Self {
            impact_threshold,
            accel_config,
            gyro_config,
            camera_model_nf,
            camera_model_wf,
            stereo_iso,
        })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&[self.impact_threshold]);
        self.accel_config.serialize(buf);
        self.gyro_config.serialize(buf);
        MinimalCameraCalibrationParams::from(self.camera_model_nf.clone()).serialize(buf);
        MinimalCameraCalibrationParams::from(self.camera_model_wf.clone()).serialize(buf);
        MinimalStereoCalibrationParams::from(self.stereo_iso).serialize(buf);
        buf.push(0); // padding
    }
}

impl Props {
    pub fn parse(bytes: &mut &[u8], _pkt_ty: PacketType) -> Result<Self, Error> {
        let mut uuid = [0; 6];
        uuid.clone_from_slice(&bytes[..6]);
        *bytes = &bytes[..6];
        Ok(Self { uuid })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&self.uuid);
    }
}

impl MotData {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        let reg0 = bytes[0];
        let id = (reg0 >> 4) & 0x0F;
        let avg_brightness = bytes[1];

        let x_raw = ((bytes[3] as u16) << 8) | (bytes[2] as u16);
        let x = x_raw as f32 / 64.0;

        let y_raw = ((bytes[5] as u16) << 8) | (bytes[4] as u16);
        let y = y_raw as f32 / 64.0;

        let area = ((bytes[7] as u16) << 8) | (bytes[6] as u16);

        *bytes = &bytes[8..];

        Ok(Self {
            id,
            area,
            avg_brightness,
            x,
            y,
        })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        let reg0 = ((self.id & 0x0F) << 4) as u8;
        let reg1 = self.avg_brightness;

        let x_raw = (self.x * 64.0).round() as u16;
        let y_raw = (self.y * 64.0).round() as u16;

        let x_low  = (x_raw & 0xFF) as u8;
        let x_high = ((x_raw >> 8) & 0xFF) as u8;
        let y_low  = (y_raw & 0xFF) as u8;
        let y_high = ((y_raw >> 8) & 0xFF) as u8;

        let area_low  = (self.area & 0xFF) as u8;
        let area_high = ((self.area >> 8) & 0xFF) as u8;

        buf.extend_from_slice(&[
            reg0, reg1,
            x_low, x_high,
            y_low, y_high,
            area_low, area_high,
        ]);
    }
}

impl ObjectReport {
    const SIZE: u16 = 112;
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
            mot_data: [(); 16].map(|_| MotData::parse(data).expect("MotData parse error")),
        })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&self.timestamp.to_le_bytes());
        for i in 0..16 {
            self.mot_data[i].serialize(buf);
        }
        buf.extend_from_slice(&[1, 0]);
    }
}

impl MarkersReport {
    const SIZE: u16 = 64;

    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        use Error as E;
        let size = Self::SIZE as usize;
        if bytes.len() < size {
            return Err(E::UnexpectedEof {
                packet_type: Some(PacketType::MarkersReport()),
            });
        }
        let data = &mut &bytes[..size];
        *bytes = &bytes[size..];
        let mut points = [Point2::new(0.0, 0.0); 16];
        for p in points.iter_mut() {
            let x_raw = u16::from_le_bytes([data[0], data[1]]);
            let y_raw = u16::from_le_bytes([data[2], data[3]]);
            let x = x_raw as f32 / 64.0;
            let y = y_raw as f32 / 64.0;
            *p = Point2::new(x, y);
            *data = &data[4..];
        }
        Ok(Self { points })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        for p in &self.points {
            let x_raw = (p.x * 64.0).round() as u16;
            let y_raw = (p.y * 64.0).round() as u16;
            buf.extend_from_slice(&x_raw.to_le_bytes());
            buf.extend_from_slice(&y_raw.to_le_bytes());
        }
    }
}

#[cfg(feature = "pyo3")]
#[pyo3::pymethods]
impl MarkersReport {
    #[getter]
    fn points(&self) -> [[f32; 2]; 16] {
        self.points.map(|p| [p.x, p.y])
    }
}

impl AccelReport {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        // accel: x, y, z, 16384 = 1g
        // gyro: x, y, z, 16.4 = 1dps
        let data = &mut &bytes[..16];
        let timestamp = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        *data = &data[4..];
        let accel = [(); 3].map(|_| {
            let x = i16::from_le_bytes([data[0], data[1]]);
            let x = (x as f32 / 2048.0) * 9.81;
            *data = &data[2..];
            x
        });
        let gyro = [(); 3].map(|_| {
            let x = i16::from_le_bytes([data[0], data[1]]);
            let x = (x as f32 / 16.4).to_radians();
            *data = &data[2..];
            x
        });
        *bytes = &bytes[16..];
        Ok(Self {
            accel: Vector3::from(accel),
            gyro: Vector3::from(gyro),
            timestamp,
        })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&self.timestamp.to_le_bytes());
        for a in self.accel.iter() {
            let a = (a / 9.81 * 2048.0) as i16;
            buf.extend_from_slice(&a.to_le_bytes());
        }
        for g in self.gyro.iter() {
            let g = (g.to_degrees() * 16.4) as i16;
            buf.extend_from_slice(&g.to_le_bytes());
        }
    }

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

impl ImpactReport {
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        let timestamp = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        *bytes = &bytes[4..];
        Ok(Self { timestamp })
    }
    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&self.timestamp.to_le_bytes());
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
