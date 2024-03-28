use ahrs::Madgwick;
use arrayvec::ArrayVec;
use nalgebra::{Matrix3x1, Point2, SMatrix, Matrix3};
use ats_cv::kalman::Pva2d;
use opencv_ros_camera::{Distortion, RosOpenCvIntrinsics};
use serde::Serialize;
use ats_usb::packet::MotData;

pub mod config_window;
pub mod marker_config_window;
pub mod layout_macro;
pub mod mot_runner;
pub mod run_raw_canvas;
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

    nf_points: ArrayVec<Point2<f64>, 16>,
    wf_points: ArrayVec<Point2<f64>, 16>,

    nf_radii: [u8; 4],
    wf_radii: [u8; 4],

    pub screen_id: u8,
    pub orientation: Madgwick<f64>,

    pub rotation_mat: Matrix3<f64>,
    pub translation_mat: Matrix3x1<f64>,

    pub camera_model_nf: RosOpenCvIntrinsics<f64>,
    pub camera_model_wf: RosOpenCvIntrinsics<f64>,
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
            orientation: Madgwick::new(1./800., 0.1),
            rotation_mat: Default::default(),
            translation_mat: Default::default(),
            camera_model_nf: RosOpenCvIntrinsics::from_params_with_distortion(
                6070.516352962917,     // fx 
                0.0,
                6081.704092101642,     // fy 
                2012.5340172306787,    // cx 
                2053.6576652187186,    // cy 
                Distortion::from_opencv_vec(nalgebra::Vector5::new(
                    -0.08717430574570494,  // k1 
                    0.5130932472490415,    // k2 
                    0.0050450411569348645, // p1 
                    0.001854950091801636,  // p2 
                    0.048480928208383456,  // k3 
                )),
            ),
            camera_model_wf: RosOpenCvIntrinsics::from_params_with_distortion(
                1404.8062660899243,    // fx 
                0.0,
                1410.4027244280476,    // fy 
                2005.7344146664623,    // cx 
                2183.296028581406,     // cy 
                Distortion::from_opencv_vec(nalgebra::Vector5::new(
                    0.03580102538774698,   // k1
                    -0.035135117459232,    // k2
                    -0.0003794652252171486,// p1
                    -0.0009616308141127135,// p2
                    0.004173283533065006,  // k3
                )),
            ),
            nf_points: Default::default(),
            wf_points: Default::default(),
            nf_radii: Default::default(),
            wf_radii: Default::default(),
        }
    }
}
