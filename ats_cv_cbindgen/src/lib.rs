#![cfg_attr(target_os = "none", no_std)]

#[cfg(target_os = "none")]
use panic_rtt_target as _;

#[repr(C)]
#[derive(Debug, PartialEq)]
pub struct Pointf32 {
    x: f32,
    y: f32,
}

impl From<Pointf32> for nalgebra::Point2<f32> {
    fn from(p: Pointf32) -> Self {
        nalgebra::Point2::new(p.x, p.y)
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
