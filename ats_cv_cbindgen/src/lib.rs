#![no_std]

#[cfg(not(test))]
use panic_rtt_target as _;

#[repr(C)]
#[derive(Debug)]
#[derive(PartialEq)]
pub struct Pointf32 {
    x: f32,
    y: f32,
}

#[no_mangle]
pub extern "C" fn transform_aim_point_f32(aim_point: Pointf32, p1: Pointf32, p2: Pointf32, p3: Pointf32, p4: Pointf32,
                                      np1: Pointf32, np2: Pointf32, np3: Pointf32, np4: Pointf32,
                                      result: &mut Pointf32) -> bool {

    match ats_cv::transform_aim_point(nalgebra::Point2::new(aim_point.x, aim_point.y),
                                      nalgebra::Point2::new(p1.x, p1.y),
                                      nalgebra::Point2::new(p2.x, p2.y),
                                      nalgebra::Point2::new(p3.x, p3.y),
                                      nalgebra::Point2::new(p4.x, p4.y),
                                      nalgebra::Point2::new(np1.x, np1.y),
                                      nalgebra::Point2::new(np2.x, np2.y),
                                      nalgebra::Point2::new(np3.x, np3.y),
                                      nalgebra::Point2::new(np4.x, np4.y)) {
        None => false,
        Some(p) => { result.x = p.x; result.y = p.y; true }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn center_aimpoint() {
        let center_aim = Pointf32 { x: 0.5, y: 0.5 };

        let p1 = Pointf32 { x: 0.5, y: 1. };
        let p2 = Pointf32 { x: 0., y: 0.5 };
        let p3 = Pointf32 { x: 0.5, y: 0. };
        let p4 = Pointf32 { x: 1., y: 0.5 };

        let mut result = Pointf32 { x: 0., y: 0. };

        transform_aim_point_f32(center_aim, p1, p2, p3, p4,
                                         Pointf32 { x: 0.5, y: 1.}, Pointf32 { x: 0., y: 0.5},
                                         Pointf32 { x: 0.5, y: 0. }, Pointf32 { x: 1., y: 0.5 },
                                         &mut result);

        assert_eq!(result, Pointf32 { x: 0.5, y: 0.5 });
    }
}
