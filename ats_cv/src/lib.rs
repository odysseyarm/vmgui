// #![no_std]

extern crate alloc;

use alloc::vec;
use nalgebra::{ComplexField, Matrix3, Point2, Point3, RealField, SMatrix, SVector, Vector2};
pub mod kalman;

use num::{traits::{float::TotalOrder, Float}, FromPrimitive};
use sqpnp::types::SolverParameters;

pub fn pnp(_3dpoints: &[Point3<f64>], _projections: &[Vector2<f64>])
{
    let solver = sqpnp::PnpSolver::new(&_3dpoints, &_projections, vec![], SolverParameters::default());
    if let Some(mut solver) = solver {
        solver.Solve();
        println!("SQPnP found {} solution(s)", solver.NumberOfSolutions());
        for i in 0..solver.NumberOfSolutions() {
            println!("Solution {i}");
            solver.SolutionPtr(i).unwrap().print();
            println!(" Average squared projection error : {:e}", solver.AverageSquaredProjectionErrors()[i]);
        }
    }
}

pub fn transform_aim_point<F: Float + FromPrimitive + 'static>(
    aim_point: Point2<F>,
    p1: Point2<F>,
    p2: Point2<F>,
    p3: Point2<F>,
    p4: Point2<F>,
    np1: Point2<F>,
    np2: Point2<F>,
    np3: Point2<F>,
    np4: Point2<F>,
) -> Option<Point2<F>>
where
    F: ComplexField,
    F: RealField,
{
    Some(get_perspective_transform(p1, p2, p3, p4, np1, np2, np3, np4)?.transform_point(&aim_point))
}

pub fn get_perspective_transform<F: Float + FromPrimitive + 'static>(
    p1: Point2<F>,
    p2: Point2<F>,
    p3: Point2<F>,
    p4: Point2<F>,
    np1: Point2<F>,
    np2: Point2<F>,
    np3: Point2<F>,
    np4: Point2<F>,
) -> Option<SMatrix<F, 3, 3>>
where
    F: ComplexField,
    F: RealField,
{
    #[rustfmt::skip]
    let a = SMatrix::<F, 8, 8>::from_row_slice(&[
        p1.x, p1.y, F::one(), F::zero(), F::zero(), F::zero(), -np1.x * p1.x, -np1.x * p1.y,
        F::zero(), F::zero(), F::zero(), p1.x, p1.y, F::one(), -np1.y * p1.x, -np1.y * p1.y,
        p2.x, p2.y, F::one(), F::zero(), F::zero(), F::zero(), -np2.x * p2.x, -np2.x * p2.y,
        F::zero(), F::zero(), F::zero(), p2.x, p2.y, F::one(), -np2.y * p2.x, -np2.y * p2.y,
        p3.x, p3.y, F::one(), F::zero(), F::zero(), F::zero(), -np3.x * p3.x, -np3.x * p3.y,
        F::zero(), F::zero(), F::zero(), p3.x, p3.y, F::one(), -np3.y * p3.x, -np3.y * p3.y,
        p4.x, p4.y, F::one(), F::zero(), F::zero(), F::zero(), -np4.x * p4.x, -np4.x * p4.y,
        F::zero(), F::zero(), F::zero(), p4.x, p4.y, F::one(), -np4.y * p4.x, -np4.y * p4.y,
    ]);
    let axp = SVector::from([np1.x, np1.y, np2.x, np2.y, np3.x, np3.y, np4.x, np4.y]);
    let decomp = a.lu();
    let p = decomp.solve(&axp)?;
    Some(Matrix3::new(p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], F::one()))
}

fn split_column<F: Float + FromPrimitive + 'static>(p: &mut [Point2<F>], threshold: F) -> (&mut [Point2<F>], &mut [Point2<F>])
where
    F: ComplexField,
    F: RealField,
{
    let [first, rest @ ..] = p else { return (&mut [], p) };
    for i in 0..rest.len() {
        if Float::abs(rest[i].x - first.x) > threshold {
            return p.split_at_mut(i + 1);
        }
    }
    (p, &mut [])
}

pub fn choose_rectangle_nearfield_markers<F: Float + FromPrimitive + 'static>(p: &mut [Point2<F>], screen_id: u8) -> Option<[Point2<F>; 4]>
where
    F: ComplexField + RealField + TotalOrder,
{
    if screen_id == 0 {
        p.sort_unstable_by(|a, b| a.x.total_cmp(&b.x));
    } else {
        p.sort_unstable_by(|a, b| b.x.total_cmp(&a.x));
    }
    let column_threshold = F::from(300).unwrap();
    let n2048 = F::from(2048).unwrap();
    let (col1, rest) = split_column(p, column_threshold);
    let (col2, _) = split_column(rest, column_threshold);
    let &col1_down = col1.iter().filter(|p| p.y > n2048).min_by(|a, b| a.y.total_cmp(&b.y))?;
    let &col1_up = col1.iter().filter(|p| p.y < n2048).max_by(|a, b| a.y.total_cmp(&b.y))?;
    let &col2_down = col2.iter().filter(|p| p.y > n2048).min_by(|a, b| a.y.total_cmp(&b.y))?;
    let &col2_up = col2.iter().filter(|p| p.y < n2048).max_by(|a, b| a.y.total_cmp(&b.y))?;
    Some([col1_up, col2_up, col2_down, col1_down])
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

        let result = transform_aim_point(
            center_aim,
            p1,
            p2,
            p3,
            p4,
            Point2::new(0.5, 1.),
            Point2::new(0., 0.5),
            Point2::new(0.5, 0.),
            Point2::new(1., 0.5),
        );

        assert_eq!(result, Some(Point2::new(0.5, 0.5)));
    }

    #[test]
    fn side_on_transform_aimpoint() {
        let r = transform_aim_point(
            Point2::new(1976., 808.),
            Point2::new(1768., 637.),
            Point2::new(353., 2583.),
            Point2::new(2834., 665.),
            Point2::new(4173., 2652.),
            Point2::new(0., 0.),
            Point2::new(8., 0.),
            Point2::new(0., 4.),
            Point2::new(8., 4.),
        )
        .unwrap();
        assert!(f64::abs(r.x - 2.) < 0.03);
        assert!(f64::abs(r.y - 1.) < 0.03);
    }

    #[test]
    fn columns() {
        let mut p = &mut [
            Point2::new(600., 3019.),
            Point2::new(653., 1442.),
            Point2::new(713., 799.),
            Point2::new(1481., 3082.),
            Point2::new(1529., 1479.),
            Point2::new(1599., 978.),
            Point2::new(2181., 1563.),
            Point2::new(2221., 869.),
            Point2::new(2231., 3154.),
            Point2::new(2927., 3168.),
            Point2::new(2946., 1555.),
            Point2::new(2967., 947.),
        ][..];
        let mut cnt = 0;
        loop {
            let (a, b) = split_column(p, 400.);
            assert_eq!(a.len(), 3, "{a:?}, {b:?}");
            p = b;
            cnt += 1;
            if p.is_empty() {
                break;
            }
        }
        assert_eq!(cnt, 4);
    }

    #[test]
    fn rectangle_screen_0() {
        let mut p = [
            Point2::new(713., 799.),
            Point2::new(1599., 978.),
            Point2::new(1529., 1479.),
            Point2::new(2927., 3168.),
            Point2::new(653., 1442.),
            Point2::new(2946., 1555.),
            Point2::new(2967., 947.),
            Point2::new(1481., 3082.),
            Point2::new(2221., 869.),
            Point2::new(600., 3019.),
            Point2::new(2181., 1563.),
            Point2::new(2231., 3154.),
        ];
        let x = choose_rectangle_nearfield_markers(&mut p, 0).unwrap();
        assert_eq!(x, [
            Point2::new(653., 1442.),
            Point2::new(1529., 1479.),
            Point2::new(1481., 3082.),
            Point2::new(600., 3019.),
        ]);
    }

    #[test]
    fn rectangle_screen_1() {
        let mut p = [
            Point2::new(713., 799.),
            Point2::new(1599., 978.),
            Point2::new(1529., 1479.),
            Point2::new(2927., 3168.),
            Point2::new(653., 1442.),
            Point2::new(2946., 1555.),
            Point2::new(2967., 947.),
            Point2::new(1481., 3082.),
            Point2::new(2221., 869.),
            Point2::new(600., 3019.),
            Point2::new(2181., 1563.),
            Point2::new(2231., 3154.),
        ];
        let x = choose_rectangle_nearfield_markers(&mut p, 1).unwrap();
        assert_eq!(x, [
            Point2::new(2946., 1555.),
            Point2::new(2181., 1563.),
            Point2::new(2231., 3154.),
            Point2::new(2927., 3168.),
        ]);
    }
}
