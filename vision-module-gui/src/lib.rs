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

#[derive(serde::Serialize, serde::Deserialize)]
#[derive(Clone, Copy, Debug)]
pub struct ScreenInfo {
    pub screen_dimensions_meters: [f64; 2],
    pub marker_points: [nalgebra::Point3<f64>; 6],
}

#[derive(serde::Deserialize)]
pub struct Plane {
    pub origin: nalgebra::Point3<f32>,
    pub normal: nalgebra::Vector3<f32>,
}

#[derive(serde::Deserialize)]
pub struct ScreenCalibration {
    plane: Plane,
    homography: nalgebra::Matrix3<f32>,
    object_points: [nalgebra::Point3<f32>; 6],
}

impl ScreenCalibration {
    fn xy_transform(&self) -> nalgebra::Transform3<f32> {
        let z_axis = self.plane.normal.normalize();
        let x_axis = nalgebra::Vector3::new(0.0, 1.0, 0.0).cross(&z_axis);
        let x_axis = if x_axis.norm() == 0.0 {
            nalgebra::Vector3::new(1.0, 0.0, 0.0).cross(&z_axis)
        } else {
            x_axis
        };
        let x_axis = x_axis.normalize();
        let y_axis = z_axis.cross(&x_axis);

        let rotation_matrix = Matrix3::from_columns(&[x_axis, y_axis, z_axis]);
        let translation = -rotation_matrix * (self.plane.normal * self.plane.origin.coords.dot(&self.plane.normal) / self.plane.normal.norm_squared());

        let mut transformation_matrix = nalgebra::Matrix4::identity();
        transformation_matrix.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation_matrix);
        transformation_matrix.fixed_view_mut::<3, 1>(0, 3).copy_from(&translation);

        nalgebra::Transform3::from_matrix_unchecked(transformation_matrix)
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

    pub fv_aimpoint_history: [Point2<f64>; 160],
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
            fv_aimpoint_pva2d: Pva2d::new(0.00001, 1.0),
            // fv_aimpoint_pva2d: Default::default(),
            fv_state: FoveatedAimpointState::new(),
            fv_aimpoint_history: [Point2::new(0.0, 0.0); 160],
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
            screen_id: if self.screen_id <= ats_cv::foveated::MAX_SCREEN_ID {
                Some(self.screen_id)
            } else {
                None
            },
        }
    }
}
