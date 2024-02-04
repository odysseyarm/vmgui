#![cfg_attr(target_os = "none", no_std)]

use core::mem::MaybeUninit;
use ats_cv::kalman::{Pva2d};
#[cfg(target_os = "none")]
use panic_semihosting as _;

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
