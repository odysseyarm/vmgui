use ahrs::Madgwick;
use arrayvec::ArrayVec;
use nalgebra::{Matrix3x1, Point2, SMatrix};
use ats_cv::kalman::Pva2d;
use serde::Serialize;
use crate::packet::MotData;

pub mod packet;
pub mod device;
pub mod config_window;
pub mod marker_config_window;
pub mod layout_macro;
pub mod mot_runner;
pub mod run_canvas;
pub mod test_canvas;
pub mod custom_shapes;

pub trait CloneButShorter: Clone {
    /// Use mainly for GUI code.
    fn c(&self) -> Self {
        self.clone()
    }
}

impl<T: Clone> CloneButShorter for T {}

#[derive(Serialize)]
pub struct Frame {
    pub nf_aim_point_x: Option<f64>,
    pub nf_aim_point_y: Option<f64>,
}

pub struct MotState {
    // Coordinates between 0.0 and 1.0
    pub fv_aim_point: Option<Point2<f64>>,
    pub nf_aim_point: Option<Point2<f64>>,
    pub wf_aim_point: Option<Point2<f64>>,

    pub nf_data: Option<ArrayVec<MotData, 16>>,
    pub wf_data: Option<ArrayVec<MotData, 16>>,
    pub nf_pva2ds: [Pva2d<f64>; 4],

    pub screen_id: u8,
    pub orientation: Madgwick<f64>,

    pub rotation_mat: SMatrix<f64, 9, 1>,
    pub translation_mat: Matrix3x1<f64>,
}

impl Default for MotState {
    fn default() -> Self {
        Self {
            fv_aim_point: None,
            nf_aim_point: None,
            wf_aim_point: None,
            nf_data: None,
            wf_data: None,
            nf_pva2ds: Default::default(),
            screen_id: 0,
            orientation: Madgwick::new(1./100., 0.1),
            rotation_mat: Default::default(),
            translation_mat: Default::default(),
        }
    }
}
