#![no_std]

use nalgebra::{ComplexField, Matrix3, Point2, RealField, SMatrix, SVector};

use num::{traits::Float, FromPrimitive};

pub fn transform_aim_point<F: Float + FromPrimitive + 'static>(aim_point: Point2<F>, p1: Point2<F>, p2: Point2<F>, p3: Point2<F>, p4: Point2<F>,
                       np1: Point2<F>, np2: Point2<F>, np3: Point2<F>, np4: Point2<F>) -> Option<Point2<F>> where F: ComplexField, F: RealField {

    let a = SMatrix::<F,8,8>::from_row_slice(
        &[p1.x, p1.y, F::from_f32(1.).unwrap(), F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(), -np1.x*p1.x, -np1.x*p1.y,
            F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(), p1.x, p1.y, F::from_f32(1.).unwrap(), -np1.y*p1.x, -np1.y*p1.y,
            p2.x, p2.y, F::from_f32(1.).unwrap(), F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(), -np2.x*p2.x, -np2.x*p2.y,
            F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(), p2.x, p2.y, F::from_f32(1.).unwrap(), -np2.y*p2.x, -np2.y*p2.y,
            p3.x, p3.y, F::from_f32(1.).unwrap(), F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(), -np3.x*p3.x, -np3.x*p3.y,
            F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(), p3.x, p3.y, F::from_f32(1.).unwrap(), -np3.y*p3.x, -np3.y*p3.y,
            p4.x, p4.y, F::from_f32(1.).unwrap(), F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(), -np4.x*p4.x, -np4.x*p4.y,
            F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(),  F::from_f32(0.).unwrap(), p4.x, p4.y, F::from_f32(1.).unwrap(), -np4.y*p4.x, -np4.y*p4.y],
    );
    let axp = SVector::from_row_slice(&[np1.x, np1.y, np2.x, np2.y, np3.x, np3.y, np4.x, np4.y]);
    let decomp = a.lu();
    let p = decomp.solve(&axp)?;
    let transformation = Matrix3::new(
        p[0], p[1], p[2],
        p[3], p[4], p[5],
        p[6], p[7], F::from(1.).unwrap()
    );
    Some(transformation.transform_point(&aim_point))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn center_aimpoint() {
        let center_aim = Point2::new(0.5, 0.5);

        let p1 = Point2::new(0.5, 1.);
        let p2 = Point2::new(0., 0.5);
        let p3 = Point2::new(0.5, 0.);
        let p4 = Point2::new(1., 0.5);

        let result = transform_aim_point(center_aim, p1, p2, p3, p4,
                            Point2::new(0.5, 1.), Point2::new(0., 0.5),
                            Point2::new(0.5, 0.), Point2::new(1., 0.5));

        assert_eq!(result, Some(Point2::new(0.5, 0.5)));
    }
}
