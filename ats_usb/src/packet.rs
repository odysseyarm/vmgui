use std::{fmt::Display, error::Error as StdError};

use ats_cv::ocv_types::{MinimalCameraCalibrationParams, MinimalStereoCalibrationParams};
use nalgebra::{Isometry3, Point2, Vector3};
use opencv_ros_camera::RosOpenCvIntrinsics;
use serde::{Deserialize, Serialize};
use serde_inline_default::serde_inline_default;

#[cfg(test)]
mod tests;

#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Debug)]
pub struct Packet {
    pub data: PacketData,
    pub id: u8,
}
#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Debug)]
pub enum PacketData {
    WriteRegister(WriteRegister), // a.k.a. Poke
    ReadRegister(Register), // a.k.a. Peek
    ReadRegisterResponse(ReadRegisterResponse),
    WriteConfig(GeneralConfig),
    ReadConfig(),
    ReadConfigResponse(GeneralConfig),
    ReadProps(),
    ReadPropsResponse(Props),
    ObjectReportRequest(ObjectReportRequest),
    ObjectReport(ObjectReport),
    CombinedMarkersReport(CombinedMarkersReport),
    AccelReport(AccelReport),
    ImpactReport(ImpactReport),
    StreamUpdate(StreamUpdate),
    FlashSettings(),
}

#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
pub enum StreamChoice {
    Object,
    CombinedMarkers,
    Accel,
    Impact,
}

#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug)]
pub struct Register {
    pub port: Port,
    pub bank: u8,
    pub address: u8,
}

#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug)]
pub struct WriteRegister {
    pub port: Port,
    pub bank: u8,
    pub address: u8,
    pub data: u8,
}

#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug)]
pub struct ReadRegisterResponse {
    pub bank: u8,
    pub address: u8,
    pub data: u8,
}

#[serde_inline_default]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Serialize, Deserialize)]
#[derive(Clone, Copy, Debug, PartialEq)]
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
        Ok(Self { accel_odr, b_x, b_y, b_z, s_x, s_y, s_z })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&self.accel_odr.to_le_bytes());
        for &b in &[self.b_x, self.b_y, self.b_z, self.s_x, self.s_y, self.s_z] {
            buf.extend_from_slice(&b.to_le_bytes());
        }
    }
}


#[serde_inline_default]
#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Serialize, Deserialize)]
#[derive(Clone, Copy, Debug, PartialEq)]
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
            camera_model_nf: RosOpenCvIntrinsics::from_params(145., 0., 145., 45., 45.),
            camera_model_wf: RosOpenCvIntrinsics::from_params(34., 0., 34., 45., 45.),
            stereo_iso: Isometry3::identity(),
        }
    }
}

#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug)]
pub struct ObjectReportRequest {}

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

#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ObjectReport {
    pub timestamp: u32,
    pub mot_data_nf: [MotData; 16],
    pub mot_data_wf: [MotData; 16],
}

#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CombinedMarkersReport {
    pub nf_points: [Point2<u16>; 16],
    pub wf_points: [Point2<u16>; 16],
    pub nf_screen_ids: [u8; 16],
    pub wf_screen_ids: [u8; 16],
}

#[cfg_attr(feature = "pyo3", pyo3::pyclass)]
#[derive(Clone, Copy, Debug, Default)]
pub struct AccelReport {
    pub timestamp: u32,
    pub accel: Vector3<f32>,
    pub gyro: Vector3<f32>,
}

#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug, Default)]
pub struct ImpactReport {
    pub timestamp: u32,
}

#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
#[derive(Clone, Copy, Debug)]
pub struct StreamUpdate {
    pub mask: u8,
    pub active: bool,
}

#[derive(Clone, Copy, Debug)]
pub enum Error {
    UnexpectedEof { packet_type: Option<PacketType> },
    UnrecognizedPacketId(u8),
    UnrecognizedPort,
    UnrecognizedMarkerPattern,
}

impl Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        use Error as S;
        match self {
            S::UnexpectedEof { packet_type: None } => write!(f, "unexpected eof"),
            S::UnexpectedEof { packet_type: Some(p) } => write!(f, "unexpected eof, packet id {p:?}"),
            S::UnrecognizedPacketId(id) => write!(f, "unrecognized packet id {id}"),
            S::UnrecognizedPort => write!(f, "unrecognized port"),
            S::UnrecognizedMarkerPattern => write!(f, "unrecognized marker pattern"),
        }
    }
}

impl StdError for Error {}

#[cfg_attr(feature = "pyo3", pyo3::pyclass(get_all))]
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
#[derive(Copy, Clone, Debug, enumn::N)]
pub enum PacketType {
    WriteRegister, // a.k.a. Poke
    ReadRegister,  // a.k.a. Peek
    ReadRegisterResponse,
    WriteConfig,
    ReadConfig,
    ReadConfigResponse,
    ReadProps,
    ReadPropsResponse,
    ObjectReportRequest,
    ObjectReport,
    CombinedMarkersReport,
    AccelReport,
    ImpactReport,
    StreamUpdate,
    FlashSettings,
    End,
}

impl TryFrom<u8> for PacketType {
    type Error = Error;
    fn try_from(n: u8) -> Result<Self, Self::Error> {
        Self::n(n).ok_or(Error::UnrecognizedPacketId(n))
    }
}

impl Packet {
    pub fn ty(&self) -> PacketType {
        match self.data {
            PacketData::WriteRegister(_) => PacketType::WriteRegister,
            PacketData::ReadRegister(_) => PacketType::ReadRegister,
            PacketData::ReadRegisterResponse(_) => PacketType::ReadRegisterResponse,
            PacketData::WriteConfig(_) => PacketType::WriteConfig,
            PacketData::ReadConfig() => PacketType::ReadConfig,
            PacketData::ReadConfigResponse(_) => PacketType::ReadConfigResponse,
            PacketData::ReadProps() => PacketType::ReadProps,
            PacketData::ReadPropsResponse(_) => PacketType::ReadPropsResponse,
            PacketData::ObjectReportRequest(_) => PacketType::ObjectReportRequest,
            PacketData::ObjectReport(_) => PacketType::ObjectReport,
            PacketData::CombinedMarkersReport(_) => PacketType::CombinedMarkersReport,
            PacketData::AccelReport(_) => PacketType::AccelReport,
            PacketData::ImpactReport(_) => PacketType::ImpactReport,
            PacketData::StreamUpdate(_) => PacketType::StreamUpdate,
            PacketData::FlashSettings() => PacketType::FlashSettings,
        }
    }

    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        let [words1, words2, ty, id, ..] = **bytes else {
            return Err(Error::UnexpectedEof { packet_type: None });
        };

        let words = u16::from_le_bytes([words1, words2]);
        let ty = PacketType::try_from(ty)?;

        let len = usize::from(words)*2;
        if bytes.len() < len {
            return Err(Error::UnexpectedEof { packet_type: Some(ty) });
        }
        *bytes = &bytes[4..];
        let data = match ty {
            PacketType::WriteRegister => PacketData::WriteRegister(WriteRegister::parse(bytes)?),
            PacketType::ReadRegister => PacketData::ReadRegister(Register::parse(bytes, ty)?),
            PacketType::ReadRegisterResponse => PacketData::ReadRegisterResponse(ReadRegisterResponse::parse(bytes)?),
            PacketType::WriteConfig => unimplemented!(),
            PacketType::ReadConfig => PacketData::ReadConfig(),
            PacketType::ReadConfigResponse => PacketData::ReadConfigResponse(GeneralConfig::parse(bytes, ty)?),
            PacketType::ReadProps => PacketData::ReadProps(),
            PacketType::ReadPropsResponse => PacketData::ReadPropsResponse(Props::parse(bytes, ty)?),
            PacketType::ObjectReportRequest => PacketData::ObjectReportRequest(ObjectReportRequest {}),
            PacketType::ObjectReport => PacketData::ObjectReport(ObjectReport::parse(bytes)?),
            PacketType::CombinedMarkersReport => PacketData::CombinedMarkersReport(CombinedMarkersReport::parse(bytes)?),
            PacketType::AccelReport => PacketData::AccelReport(AccelReport::parse(bytes)?),
            PacketType::ImpactReport => PacketData::ImpactReport(ImpactReport::parse(bytes)?),
            PacketType::StreamUpdate => PacketData::StreamUpdate(StreamUpdate::parse(bytes)?),
            PacketType::FlashSettings => PacketData::FlashSettings(),
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
            PacketData::ReadRegister(_) => calculate_length!(Register)+1,
            PacketData::ReadRegisterResponse(_) => calculate_length!(ReadRegisterResponse)+1,
            PacketData::WriteConfig(_) => GeneralConfig::SIZE,
            PacketData::ReadConfig() => 0,
            PacketData::ReadConfigResponse(_) => GeneralConfig::SIZE,
            PacketData::ReadProps() => 0,
            PacketData::ReadPropsResponse(_) => 6,
            PacketData::ObjectReportRequest(_) => calculate_length!(ObjectReportRequest),
            PacketData::ObjectReport(_) => ObjectReport::SIZE,
            PacketData::CombinedMarkersReport(_) => CombinedMarkersReport::SIZE,
            PacketData::AccelReport(_) => 16,
            PacketData::ImpactReport(_) => 4,
            PacketData::StreamUpdate(_) => calculate_length!(StreamUpdate),
            PacketData::FlashSettings() => 0,
        };
        let words = u16::to_le_bytes((len + 4) / 2);
        let ty = self.ty();
        buf.reserve(4 + usize::from(len));
        buf.extend_from_slice(&[words[0], words[1], ty as u8, self.id]);
        match &self.data {
            PacketData::WriteRegister(x) => x.serialize(buf),
            PacketData::ReadRegister(x) => x.serialize(buf),
            PacketData::ReadRegisterResponse(x) => x.serialize(buf),
            PacketData::WriteConfig(x) => x.serialize(buf),
            PacketData::ReadConfig() => (),
            PacketData::ReadConfigResponse(x) => x.serialize(buf),
            PacketData::ReadProps() => (),
            PacketData::ReadPropsResponse(x) => x.serialize(buf),
            PacketData::ObjectReportRequest(_) => (),
            PacketData::ObjectReport(x) => x.serialize(buf),
            PacketData::CombinedMarkersReport(x) => x.serialize(buf),
            PacketData::AccelReport(x) => x.serialize(buf),
            PacketData::ImpactReport(x) => x.serialize(buf),
            PacketData::StreamUpdate(x) => buf.extend_from_slice(&[x.mask as u8, x.active as u8]),
            PacketData::FlashSettings() => (),
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

impl Register {
    pub fn parse(bytes: &mut &[u8], pkt_ty: PacketType) -> Result<Self, Error> {
        use Error as E;
        let [port, bank, address, _, ..] = **bytes else {
            return Err(E::UnexpectedEof { packet_type: Some(pkt_ty) });
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
            return Err(E::UnexpectedEof { packet_type: Some(PacketType::ReadRegisterResponse) });
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
            return Err(E::UnexpectedEof { packet_type: Some(PacketType::WriteRegister) });
        };
        let port = port.try_into()?;
        *bytes = &bytes[4..];
        Ok(Self { port, bank, address, data })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&[self.port as u8, self.bank, self.address, self.data]);
    }
}

impl GeneralConfig {
    pub const SIZE: u16 = 200;
    pub fn parse(bytes: &mut &[u8], pkt_ty: PacketType) -> Result<Self, Error> {
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

        let mut camera_model_nf: RosOpenCvIntrinsics<f32> = match ats_cv::ocv_types::MinimalCameraCalibrationParams::parse(bytes) {
            Ok(x) => x.into(),
            Err(_) => return Err(E::UnexpectedEof { packet_type: None }),
        };

        let mut camera_model_wf: RosOpenCvIntrinsics<f32> = match ats_cv::ocv_types::MinimalCameraCalibrationParams::parse(bytes) {
            Ok(x) => x.into(),
            Err(_) => return Err(E::UnexpectedEof { packet_type: None }),
        };

        let stereo_iso = match ats_cv::ocv_types::MinimalStereoCalibrationParams::parse(bytes) {
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

        Ok(Self { impact_threshold, accel_config, gyro_config, camera_model_nf, camera_model_wf, stereo_iso })
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
    pub fn parse(bytes: &mut &[u8], pkt_ty: PacketType) -> Result<Self, Error> {
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
            return Err(E::UnexpectedEof { packet_type: Some(PacketType::ObjectReport) });
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
    const SIZE: u16 = 108;
    pub fn parse(bytes: &mut &[u8]) -> Result<Self, Error> {
        use Error as E;
        let size = Self::SIZE as usize;
        if bytes.len() < size {
            return Err(E::UnexpectedEof { packet_type: Some(PacketType::CombinedMarkersReport) });
        }

        let data = &mut &bytes[..size];
        *bytes = &bytes[size..];

        let mut positions = [Point2::new(0, 0); 16*2];
        for i in 0..positions.len() {
            // x, y is 12 bits each
            let x = u16::from_le_bytes([data[0], data[1] & 0x0f]);
            let y = (data[1] >> 4) as u16 | ((data[2] as u16) << 4);
            positions[i] = Point2::new(x, y);
            *data = &data[3..];
        }
        let nf_positions = positions[..16].try_into().unwrap();
        let wf_positions = positions[16..].try_into().unwrap();

        let mut screen_ids = [0; 32];

        let mut bit_offset = 0;
        for i in 0..32 {
            let byte_index = bit_offset / 8;
            let bit_index = bit_offset % 8;

            screen_ids[i] = if bit_index <= 5 {
                // The bits are within the same byte
                (data[byte_index] >> bit_index) & 0x7
            } else {
                // The bits span across two bytes
                let first_part = data[byte_index] >> bit_index;
                let second_part = data[byte_index + 1] << (8 - bit_index);
                (first_part | second_part) & 0x7
            };

            bit_offset += 3;
        }

        *data = &data[12..];

        let nf_screen_ids = screen_ids[..16].try_into().unwrap();
        let wf_screen_ids = screen_ids[16..].try_into().unwrap();

        Ok(Self { nf_points: nf_positions, wf_points: wf_positions, nf_screen_ids, wf_screen_ids })
    }

    pub fn serialize(&self, buf: &mut Vec<u8>) {
        for p in self.nf_points.iter().chain(&self.wf_points) {
            let (x, y) = (p.x, p.y);
            let byte0 = x & 0xff;
            let byte1 = ((x >> 8) & 0x0f) | ((y & 0x0f) << 4);
            let byte2 = y >> 4;
            buf.extend_from_slice(&[byte0 as u8, byte1 as u8, byte2 as u8]);
        }

        buf.extend({
            let mut buf = [0; 12];
            for i in 0..32 {
                let byte_index = (i * 3) / 8;
                let bit_index = (i * 3) % 8;
                let screen_id = if i < 16 {
                    self.nf_screen_ids[i]
                } else {
                    self.wf_screen_ids[i - 16]
                } & 0x07; // Mask to 3 bits

                let mask = screen_id << bit_index;
                buf[byte_index] |= mask;
                if bit_index > 5 {
                    buf[byte_index + 1] |= screen_id >> (8 - bit_index);
                }
            }
            buf
        });
    }
}

#[cfg(feature = "pyo3")]
#[pyo3::pymethods]
impl CombinedMarkersReport {
    #[getter]
    fn nf_screen_ids(&self) -> [u8; 16] {
        self.nf_screen_ids
    }
    #[getter]
    fn wf_screen_ids(&self) -> [u8; 16] {
        self.wf_screen_ids
    }
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
        Ok(Self { accel: Vector3::from(accel), gyro: Vector3::from(gyro), timestamp })
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
            mask: bytes[0],
            active: bytes[1] != 0,
        };
        *bytes = &bytes[2..];
        Ok(stream_update)
    }
}
