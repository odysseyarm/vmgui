use ahrs::Madgwick;
use arrayvec::ArrayVec;
use nalgebra::{Isometry3, Matrix3, Matrix3x1, Point2};
use ats_cv::kalman::Pva2d;
use opencv_ros_camera::RosOpenCvIntrinsics;
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
pub mod tracking_canvas_helpers;

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
    pub fv_aim_point: Point2<f64>,
    pub nf_aim_point: Point2<f64>,
    pub wf_aim_point: Point2<f64>,

    pub nf_data: Option<ArrayVec<MotData, 16>>,
    pub wf_data: Option<ArrayVec<MotData, 16>>,

    nf_points: ArrayVec<Point2<f64>, 16>,
    wf_points: ArrayVec<Point2<f64>, 16>,
    pub nf_pva2ds: [Pva2d<f64>; 16],
    pub wf_pva2ds: [Pva2d<f64>; 16],

    nf_radii: [u8; 4],
    wf_radii: [u8; 4],

    pub screen_id: u8,
    pub orientation: Madgwick<f64>,
    pub gravity_angle: f32,

    pub rotation_mat: Matrix3<f64>,
    pub translation_mat: Matrix3x1<f64>,

    pub camera_model_nf: RosOpenCvIntrinsics<f64>,
    pub camera_model_wf: RosOpenCvIntrinsics<f64>,
    pub stereo_iso: Isometry3<f64>,

    pub nf_aim_point_history: [Point2<f64>; 40],
    pub nf_aim_point_history_index: usize,
}

impl Default for MotState {
    fn default() -> Self {
        let nf_default_yaml = b"
            camera_matrix: !!opencv-matrix
                rows: 3
                cols: 3
                dt: d
                data: [ 145.10303635182407, 0., 47.917007513463489, 0.,
                    145.29149528441428, 49.399597700110256, 0., 0., 1. ]
            dist_coeffs: !!opencv-matrix
                rows: 1
                cols: 5
                dt: d
                data: [ -0.20386528104463086, 2.2361997667805928,
                    0.0058579118546963271, -0.0013804251043507982,
                    -7.7712738787306455 ]
            rms_error: 0.074045711900311811
            num_captures: 62
        ";

        let wf_default_yaml = b"
            camera_matrix: !!opencv-matrix
               rows: 3
               cols: 3
               dt: d
               data: [ 34.34121647200962, 0., 48.738547789766642, 0.,
                   34.394866762375322, 49.988446965249153, 0., 0., 1. ]
            dist_coeffs: !!opencv-matrix
               rows: 1
               cols: 5
               dt: d
               data: [ 0.039820534469617412, -0.039933169314557031,
                   0.00043006078813049756, -0.0012057066028621883,
                   0.0053022349797757964 ]
            rms_error: 0.066816050332039037
            num_captures: 64
        ";

        let stereo_default_yaml = b"
            r: !!opencv-matrix
                rows: 3
                cols: 3
                dt: d
                data: [ 0.99998692169365289, -0.0051111539633757353,
                    -0.00018040735794555248, 0.0050780667355570033,
                    0.99647084468055069, -0.083785851668757322,
                    0.00060801306019016873, 0.083783839771118418, 0.99648376731049981 ]
            t: !!opencv-matrix
                rows: 3
                cols: 1
                dt: d
                data: [ -0.011673356870756095, -0.33280456937540659,
                    0.19656043337961257 ]
            rms_error: 0.1771451374078685
            num_captures: 49
        ";

        Self {
            fv_aim_point: Point2::new(0.0, 0.0),
            nf_aim_point: Point2::new(0.0, 0.0),
            wf_aim_point: Point2::new(0.0, 0.0),
            nf_data: None,
            wf_data: None,
            nf_pva2ds: Default::default(),
            wf_pva2ds: Default::default(),
            screen_id: 0,
            orientation: Madgwick::new(1./800., 0.1),
            gravity_angle: 0.,
            rotation_mat: Default::default(),
            translation_mat: Default::default(),
            camera_model_nf: ats_cv::get_intrinsics_from_opencv_camera_calibration_yaml(&nf_default_yaml[..]).unwrap(),
            camera_model_wf: ats_cv::get_intrinsics_from_opencv_camera_calibration_yaml(&wf_default_yaml[..]).unwrap(),
            stereo_iso: ats_cv::get_isometry_from_opencv_stereo_calibration_yaml(&stereo_default_yaml[..]).unwrap(),
            nf_points: Default::default(),
            wf_points: Default::default(),
            nf_radii: Default::default(),
            wf_radii: Default::default(),
            nf_aim_point_history: [Point2::new(0.0, 0.0); 40],
            nf_aim_point_history_index: 0,
        }
    }
}
