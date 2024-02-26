#![cfg_attr(target_os = "none", no_std)]

use core::mem::MaybeUninit;
use ahrs::{Ahrs as _, Madgwick};
use ats_cv::kalman::Pva2d;
use cam_geom::Pixels;
use nalgebra::{Matrix, Point2, Vector3, Vector5};
use opencv_ros_camera::{Distortion, RosOpenCvIntrinsics};
#[cfg(target_os = "none")]
use panic_semihosting as _;

#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Pointf32 {
    x: f32,
    y: f32,
}

impl From<Pointf32> for nalgebra::Point2<f32> {
    fn from(p: Pointf32) -> Self {
        nalgebra::Point2::new(p.x, p.y)
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Point3f32 {
    x: f32,
    y: f32,
    z: f32,
}

impl From<Vector3<f32>> for Point3f32 {
    fn from(p: Vector3<f32>) -> Self {
        Self {
            x: p.x,
            y: p.y,
            z: p.z,
        }
    }
}

#[no_mangle]
pub extern "C" fn transform_aim_point_f32(
    aim_point: Pointf32,
    p1: Pointf32,
    p2: Pointf32,
    p3: Pointf32,
    p4: Pointf32,
    np1: Pointf32,
    np2: Pointf32,
    np3: Pointf32,
    np4: Pointf32,
    result: &mut Pointf32,
) -> bool {
    match ats_cv::transform_aim_point(
        aim_point.into(),
        p1.into(),
        p2.into(),
        p3.into(),
        p4.into(),
        np1.into(),
        np2.into(),
        np3.into(),
        np4.into(),
    ) {
        None => false,
        Some(p) => {
            result.x = p.x;
            result.y = p.y;
            true
        }
    }
}

/// Result must have space for 4 points. Returns true on success
#[no_mangle]
pub extern "C" fn choose_rectangle_nearfield_markers_f32(p: *mut Pointf32, len: usize, screen_id: u8, result: *mut Pointf32) -> bool {
    let p = unsafe { core::slice::from_raw_parts_mut(p as *mut Point2<f32>, len) };
    let result = unsafe { core::slice::from_raw_parts_mut(result as *mut Point2<f32>, 4) };
    let r = ats_cv::choose_rectangle_nearfield_markers::<f32>(p, screen_id);
    if let Some(r) = r {
        result.copy_from_slice(&r);
        true
    } else {
        false
    }
}

/// Simple kalman filter for position, velocity, and acceleration assuming constant acceleration
#[repr(C)]
pub union Pva2d_f32 {
    pub _size: [u8; 176],
    pub _align: u32,
}
static_assertions::assert_eq_size!(Pva2d_f32, Pva2d<f32>);
static_assertions::assert_eq_align!(Pva2d_f32, Pva2d<f32>);

#[no_mangle]
pub extern "C" fn pva2df32_create_default(pva2df32: &mut MaybeUninit<Pva2d<f32>>) {
    pva2df32.write(Pva2d::default());
}

#[no_mangle]
pub extern "C" fn pva2df32_observe(pva2df32: &mut Pva2d<f32>, px: f32, py: f32, vx: f32, vy: f32) {
    pva2df32.observe(&[px, py], &[vx, vy]);
}

#[no_mangle]
pub extern "C" fn pva2df32_step(pva2df32: &mut Pva2d<f32>) {
    pva2df32.step();
}

#[no_mangle]
pub extern "C" fn pva2df32_position(pva2df32: &mut Pva2d<f32>, px: &mut MaybeUninit<f32>, py: &mut MaybeUninit<f32>) {
    let pos = pva2df32.position();
    px.write(pos[0]);
    py.write(pos[1]);
}

#[repr(C)]
pub union CameraModel {
    pub _size: [u8; 264],
    pub _align: u32,
}

static_assertions::assert_eq_size!(CameraModel, RosOpenCvIntrinsics<f32>);
static_assertions::assert_eq_align!(CameraModel, RosOpenCvIntrinsics<f32>);

impl CameraModel {
    fn as_ros_mut(p: *mut Self) -> *mut RosOpenCvIntrinsics<f32> {
        p as _
    }
    fn as_ros(p: *const Self) -> *const RosOpenCvIntrinsics<f32> {
        p as _
    }
}

#[no_mangle]
pub extern "C" fn ats_cv_create_camera_model(
    fx: f32, fy: f32, cx: f32, cy: f32,
    k1: f32, k2: f32, p1: f32, p2: f32, k3: f32,
    result: *mut CameraModel,
) {
    let result = CameraModel::as_ros_mut(result);
    unsafe {
        result.write(RosOpenCvIntrinsics::from_params_with_distortion(fx, 0.0, fy, cx, cy, Distortion::from_opencv_vec(Vector5::new(k1, k2, p1, p2, k3))));
    }
}

#[no_mangle]
pub extern "C" fn ats_cv_camera_undistort(
    camera_model: *const CameraModel,
    x: f32, y: f32,
    rx: *mut f32, ry: *mut f32,
) {
    // TODO don't make references
    let camera_model = unsafe { &*CameraModel::as_ros(camera_model) };
    let distorted = Pixels::new(Matrix::from([[x], [y]]));
    let undistorted = camera_model.undistort(&distorted);
    unsafe {
        rx.write(undistorted.data.x);
        ry.write(undistorted.data.y);
    }
}

/// Undistort 4 coordinates at once
#[no_mangle]
pub extern "C" fn ats_cv_camera_undistort_4(
    camera_model: *const CameraModel,
    x: *const [f32; 4], y: *const [f32; 4],
    rx: *mut [f32; 4], ry: *mut [f32; 4],
) {
    let camera_model = unsafe { &*CameraModel::as_ros(camera_model) };
    let x = unsafe { x.read() };
    let y = unsafe { y.read() };
    let distorted = Pixels::new(Matrix::from([x, y]));
    let undistorted = camera_model.undistort(&distorted);
    unsafe {
        rx.write(undistorted.data.data.0[0]);
        ry.write(undistorted.data.data.0[1]);
    }
}

#[repr(C)]
pub union AtsCvMadgwick {
    pub _size: [u8; 24],
    pub _align: u32,
}

static_assertions::assert_eq_size!(AtsCvMadgwick, Madgwick<f32>);
static_assertions::assert_eq_align!(AtsCvMadgwick, Madgwick<f32>);

impl AtsCvMadgwick {
    fn as_madgwick_mut(p: *mut Self) -> *mut Madgwick<f32> {
        p as _
    }
    fn as_madgwick(p: *const Self) -> *const Madgwick<f32> {
        p as _
    }
}
#[no_mangle]
pub extern "C" fn ats_cv_madgwick_init(
    period: f32,
    beta: f32,
    p: *mut AtsCvMadgwick,
) {
    let p = AtsCvMadgwick::as_madgwick_mut(p);
    unsafe { p.write(Madgwick::new(period, beta)); }
}

/// Returns false on failure.
///
/// Failure occurs when the norm of the accelerometer measurement is zero.
#[no_mangle]
pub extern "C" fn ats_cv_madgwick_update_imu(
    ax: f32,
    ay: f32,
    az: f32,
    gx: f32,
    gy: f32,
    gz: f32,
    p: *mut AtsCvMadgwick,
) -> bool {
    let p = unsafe { &mut *AtsCvMadgwick::as_madgwick_mut(p) };
    p.update_imu(&Vector3::new(gx, gy, gz), &Vector3::new(ax, ay, az)).is_ok()
}

/// Calculate gravity vector.
#[no_mangle]
pub extern "C" fn ats_cv_madgwick_gravity_vector(p: *const AtsCvMadgwick) -> Point3f32 {
    let p = unsafe { &*AtsCvMadgwick::as_madgwick(p) };
    p.quat.inverse_transform_vector(&Vector3::z()).into()
}
