use arrayvec::ArrayVec;
use nalgebra::{Matrix3, Matrix3x1, Point2, Rotation3, Vector2};
use ats_cv::{foveated::FoveatedAimpointState, kalman::Pva2d};
use serde::Serialize;
use ats_usb::packet::MotData;

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

    pub fv_aimpoint_history: [Point2<f64>; 80],
    pub fv_aimpoint_history_index: usize,
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
            fv_aimpoint_pva2d: Pva2d::new(0.2, 1.0),
            // fv_aimpoint_pva2d: Default::default(),
            fv_state: FoveatedAimpointState::new(),
            fv_aimpoint_history: [Point2::new(0.0, 0.0); 80],
            fv_aimpoint_history_index: 0,
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
            screen_id: if self.screen_id < 7 {
                Some(self.screen_id)
            } else {
                None
            },
        }
    }
}
