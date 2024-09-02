use arrayvec::ArrayVec;
use ats_cv::foveated::MARKER_PATTERN_LEN;
use nalgebra::{Matrix3, Matrix3x1, Point2, Rotation3, Vector2};
use ats_cv::{foveated::FoveatedAimpointState, kalman::Pva2d};
use serde::Serialize;
use ats_usb::packet::MotData;
use sqpnp::types::SQPSolution;

pub mod config_window;
pub mod layout_macro;
pub mod mot_runner;
pub mod run_raw_canvas;
pub mod run_canvas;
pub mod test_canvas;
pub mod custom_shapes;
pub mod tracking_canvas_helpers;
pub mod plots_window;
pub mod consts;

pub trait CloneButShorter: Clone {
    /// Use mainly for GUI code.
    fn c(&self) -> Self {
        self.clone()
    }
}

impl<T: Clone> CloneButShorter for T {}

#[derive(Serialize)]
pub struct TestFrame {
    pub fv_aimpoint_x: Option<f64>,
    pub fv_aimpoint_y: Option<f64>,
}

#[derive(serde::Serialize, serde::Deserialize)]
#[derive(Clone, Copy, Debug)]
pub struct ScreenInfo {
    pub screen_dimensions_meters: [f64; 2],
    pub marker_points: [nalgebra::Point3<f64>; MARKER_PATTERN_LEN],
}

impl From<ScreenInfo> for ats_cv::ScreenCalibration<f64> {
    fn from(screen_info: ScreenInfo) -> Self {
        ats_cv::ScreenCalibration {
            homography: {
                nalgebra::Matrix3::new(
                    1./screen_info.screen_dimensions_meters[0], 0.0, 0.0,
                    0.0, 1./screen_info.screen_dimensions_meters[1], 0.0,
                    0.0, 0.0, 1.0,
                )
            },
            object_points: screen_info.marker_points,
        }
    }
}

pub struct MotState {
    // Coordinates between 0.0 and 1.0
    pub fv_aimpoint: Point2<f64>,

    pub nf_data: Option<ArrayVec<MotData, 16>>,
    pub wf_data: Option<ArrayVec<MotData, 16>>,

    pub nf_points: ArrayVec<(u8, u8, Point2<f64>), 16>,
    pub wf_points: ArrayVec<(u8, u8, Point2<f64>), 16>,
    pub nf_markers: ArrayVec<Point2<f64>, 16>,
    pub wf_markers: ArrayVec<Point2<f64>, 16>,
    pub wf_reproj: ArrayVec<Point2<f64>, 16>,
    pub nf_markers2: ArrayVec<Marker, 16>,
    pub wf_markers2: ArrayVec<Marker, 16>,

    pub fv_aimpoint_pva2d: Pva2d<f64>,

    pub screen_id: u8,
    pub orientation: Rotation3<f32>,
    pub madgwick: ahrs::Madgwick<f32>,

    pub rotation_mat: Matrix3<f64>,
    pub translation_mat: Matrix3x1<f64>,

    pub fv_state: ats_cv::foveated::FoveatedAimpointState,

    pub fv_aimpoint_history: [Point2<f64>; 100],
    pub fv_aimpoint_history_index: usize,

    pub previous_sqp_solution: Option<SQPSolution>,
}

impl Default for MotState {
    fn default() -> Self {
        Self {
            fv_aimpoint: Point2::new(0.0, 0.0),
            nf_data: None,
            wf_data: None,
            screen_id: 0,
            orientation: Rotation3::identity(),
            madgwick: ahrs::Madgwick::new(1./100., 0.1),
            rotation_mat: Default::default(),
            translation_mat: Default::default(),
            nf_points: Default::default(),
            wf_points: Default::default(),
            nf_markers: Default::default(),
            wf_markers: Default::default(),
            wf_reproj: Default::default(),
            nf_markers2: Default::default(),
            wf_markers2: Default::default(),
            fv_aimpoint_pva2d: Pva2d::new(0.0001, 1.0),
            // fv_aimpoint_pva2d: Default::default(),
            fv_state: FoveatedAimpointState::new(),
            fv_aimpoint_history: [Point2::new(0.0, 0.0); 100],
            fv_aimpoint_history_index: 0,
            previous_sqp_solution: None,
        }
    }
}

pub struct Marker {
    pub mot_id: u8,
    pub screen_id: u8,
    pub pattern_id: Option<u8>,
    pub normalized: Point2<f64>,
}

impl Marker {
    pub fn ats_cv_marker(&self) -> ats_cv::foveated::Marker {
        ats_cv::foveated::Marker {
            position: self.normalized,
            screen_id: if self.screen_id <= ats_cv::foveated::MAX_SCREEN_ID {
                Some(self.screen_id)
            } else {
                None
            },
        }
    }
}
