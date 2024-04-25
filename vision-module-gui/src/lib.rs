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

    nf_points: ArrayVec<Point2<f64>, 16>,
    wf_points: ArrayVec<Point2<f64>, 16>,
    pub nf_pva2ds: [Pva2d<f64>; 4],
    pub wf_pva2ds: [Pva2d<f64>; 4],

    nf_radii: [u8; 4],
    wf_radii: [u8; 4],

    pub screen_id: u8,
    pub orientation: Madgwick<f64>,

    pub rotation_mat: Matrix3<f64>,
    pub translation_mat: Matrix3x1<f64>,

    pub camera_model_nf: RosOpenCvIntrinsics<f64>,
    pub camera_model_wf: RosOpenCvIntrinsics<f64>,
    pub camera_model_stereo: RosOpenCvIntrinsics<f64>,
}

impl Default for MotState {
    fn default() -> Self {
        let nf_default_yaml = b"
            %YAML:1.0
            ---
            camera_matrix: !!opencv-matrix
            rows: 3
            cols: 3
            dt: d
            data: [ 151.36955641854573, 0., 46.374189382434295, 0.,
                151.67240449552887, 45.515931664067992, 0., 0., 1. ]
            dist_coeffs: !!opencv-matrix
            rows: 1
            cols: 5
            dt: d
            data: [ 0.18209478476909499, -5.6187772186705915,
                -0.0046771981466741282, -0.0091880927925765497,
                34.923550986754449 ]
            rms_error: 0.13243574013156023
            num_captures: 25
        ";

        let wf_default_yaml = b"
            %YAML:1.0
            ---
            camera_matrix: !!opencv-matrix
            rows: 3
            cols: 3
            dt: d
            data: [ 34.128736955254823, 0., 47.862507458039438, 0.,
                34.150205588747298, 49.921544503385206, 0., 0., 1. ]
            dist_coeffs: !!opencv-matrix
            rows: 1
            cols: 5
            dt: d
            data: [ 0.020886129364872441, -0.021671002882625662,
                0.0047637101829026418, -0.0040207558255908873,
                0.0021400959520238745 ]
            rms_error: 0.22294102186428852
            num_captures: 38
        ";

        let stereo_default_yaml = b"
            %YAML:1.0
            ---
            r: !!opencv-matrix
            rows: 3
            cols: 3
            dt: d
            data: [ 0.99997813502228272, -0.0045152181240831236,
                -0.0048313851687950505, 0.0043339510700983117,
                0.99930981448611589, -0.036893245179121563, 0.0049946316660285572,
                0.036871499522215447, 0.99930753333371003 ]
            t: !!opencv-matrix
            rows: 3
            cols: 1
            dt: d
            data: [ 0.027827639169328636, -0.61376953427966641,
                0.68626318075117543 ]
            rms_error: 0.18646313875884704
            num_captures: 47
        ";

        Self {
            fv_aim_point: None,
            nf_aim_point: None,
            wf_aim_point: None,
            nf_data: None,
            wf_data: None,
            nf_pva2ds: Default::default(),
            wf_pva2ds: Default::default(),
            screen_id: 0,
            orientation: Madgwick::new(1./800., 0.1),
            rotation_mat: Default::default(),
            translation_mat: Default::default(),
            camera_model_nf: ats_cv::get_intrinsics_from_opencv_yaml(&nf_default_yaml[..]).unwrap(),
            camera_model_wf: ats_cv::get_intrinsics_from_opencv_yaml(&wf_default_yaml[..]).unwrap(),
            camera_model_stereo: ats_cv::get_intrinsics_from_opencv_yaml(&stereo_default_yaml[..]).unwrap(),
            nf_points: Default::default(),
            wf_points: Default::default(),
            nf_radii: Default::default(),
            wf_radii: Default::default(),
        }
    }
}
