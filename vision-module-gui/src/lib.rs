use arrayvec::ArrayVec;
use ats_cv::foveated::FoveatedAimpointState;
use ats_common::MARKER_PATTERN_LEN;
use ats_usb::packets::vm::MotData;
use nalgebra::Isometry3;
use nalgebra::{Matrix3, Matrix3x1, Point2, Rotation3};
use serde::Serialize;

pub mod config_window;
pub mod consts;
pub mod custom_shapes;
pub mod layout_macro;
pub mod mot_runner;
pub mod plots_window;
pub mod run_canvas;
pub mod run_raw_canvas;
pub mod test_canvas;
pub mod tracking_canvas_helpers;

pub trait CloneButShorter: Clone {
    /// Use mainly for GUI code.
    fn c(&self) -> Self {
        self.clone()
    }
}

impl<T: Clone> CloneButShorter for T {}

#[derive(Serialize)]
pub struct TestFrame {
    pub fv_aimpoint_x: Option<f32>,
    pub fv_aimpoint_y: Option<f32>,
    pub opposite_cant: Option<f32>,
    pub position_x: Option<f32>,
    pub position_y: Option<f32>,
    pub position_z: Option<f32>,
}

#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, Debug)]
pub struct ScreenInfo {
    pub screen_dimensions_meters: [f32; 2],
    pub marker_points: [nalgebra::Point3<f32>; MARKER_PATTERN_LEN],
}

impl From<ScreenInfo> for ats_common::ScreenCalibration<f32> {
    fn from(screen_info: ScreenInfo) -> Self {
        ats_common::ScreenCalibration {
            rotation: Default::default(),
            homography: {
                nalgebra::Matrix3::new(
                    1. / screen_info.screen_dimensions_meters[0],
                    0.0,
                    0.0,
                    0.0,
                    1. / screen_info.screen_dimensions_meters[1],
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                )
            },
            object_points: screen_info.marker_points,
        }
    }
}

pub struct MotState {
    // Coordinates between 0.0 and 1.0
    pub fv_aimpoint: Point2<f32>,

    pub distance: f32,

    pub nf_data: Option<ArrayVec<MotData, 16>>,
    pub wf_data: Option<ArrayVec<MotData, 16>>,

    pub nf_points: ArrayVec<(u8, Point2<f32>), 16>,
    pub wf_points: ArrayVec<(u8, Point2<f32>), 16>,
    pub nf_markers: ArrayVec<Point2<f32>, 16>,
    pub wf_markers: ArrayVec<Point2<f32>, 16>,
    pub wf_reproj: ArrayVec<Point2<f32>, 16>,
    pub nf_markers2: ArrayVec<Marker, 16>,
    pub wf_markers2: ArrayVec<Marker, 16>,

    pub screen_id: u8,
    pub orientation: Rotation3<f32>,
    pub madgwick: ahrs::Madgwick<f32>,

    pub rotation_mat: Matrix3<f32>,
    pub translation_mat: Matrix3x1<f32>,

    pub fv_state: ats_cv::foveated::FoveatedAimpointState,
    pub fv_zero_offset: Isometry3<f32>,

    pub fv_aimpoint_history: [(Point2<f32>, f32, Matrix3x1<f32>); 80],
    pub fv_aimpoint_history_index: usize,
}

impl Default for MotState {
    fn default() -> Self {
        Self {
            fv_aimpoint: Point2::new(0.0, 0.0),
            distance: 0.0,
            nf_data: None,
            wf_data: None,
            screen_id: 0,
            orientation: Rotation3::identity(),
            madgwick: ahrs::Madgwick::new(1. / 100., 0.04),
            rotation_mat: Default::default(),
            translation_mat: Default::default(),
            nf_points: Default::default(),
            wf_points: Default::default(),
            nf_markers: Default::default(),
            wf_markers: Default::default(),
            wf_reproj: Default::default(),
            nf_markers2: Default::default(),
            wf_markers2: Default::default(),
            fv_state: FoveatedAimpointState::new(),
            fv_zero_offset: Isometry3::identity(),
            fv_aimpoint_history: [(Point2::new(0.0, 0.0), 0., Matrix3x1::new(0.0, 0.0, 0.0)); 80],
            fv_aimpoint_history_index: 0,
        }
    }
}

pub struct Marker {
    pub mot_id: u8,
    pub pattern_id: Option<u8>,
    pub normalized: Point2<f32>,
}

impl Marker {
    pub fn ats_cv_marker(&self) -> ats_cv::foveated::Marker {
        ats_cv::foveated::Marker {
            position: self.normalized.cast(),
        }
    }
}
