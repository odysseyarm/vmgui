use arrayvec::ArrayVec;
use nalgebra::Point2;
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

#[derive(Default)]
pub struct MotState {
    // Coordinates between 0.0 and 1.0
    pub fv_aim_point: Option<Point2<f64>>,
    pub nf_aim_point: Option<Point2<f64>>,
    pub wf_aim_point: Option<Point2<f64>>,

    pub nf_data: Option<ArrayVec<MotData, 16>>,
    pub wf_data: Option<ArrayVec<MotData, 16>>,
    pub nf_pva2ds: [Pva2d<f64>; 4],
}
