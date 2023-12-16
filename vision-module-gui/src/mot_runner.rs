use std::mem::swap;
use std::sync::Arc;
use std::time::Duration;
use arrayvec::ArrayVec;
use nalgebra::{Matrix3, OMatrix, Point2, SMatrix, SVector, U1, U8};
use tokio::sync::Mutex;
use tokio::time::sleep;
use crate::{CloneButShorter, MotState};
use crate::device::UsbDevice;
use crate::packet::MotData;

fn transform_aim_point(aim_point: Point2<f64>, p1: Point2<f64>, p2: Point2<f64>, p3: Point2<f64>, p4: Point2<f64>,
             np1: Point2<f64>, np2: Point2<f64>, np3: Point2<f64>, np4: Point2<f64>) -> Option<Point2<f64>> {

    let a = SMatrix::<f64,8,8>::from_row_slice(
        &[p1.x, p1.y, 1., 0.,  0.,  0., -np1.x*p1.x, -np1.x*p1.y,
        0.,  0.,  0., p1.x, p1.y, 1., -np1.y*p1.x, -np1.y*p1.y,
        p2.x, p2.y, 1., 0.,  0.,  0., -np2.x*p2.x, -np2.x*p2.y,
        0.,  0.,  0., p2.x, p2.y, 1., -np2.y*p2.x, -np2.y*p2.y,
        p3.x, p3.y, 1., 0.,  0.,  0., -np3.x*p3.x, -np3.x*p3.y,
        0.,  0.,  0., p3.x, p3.y, 1., -np3.y*p3.x, -np3.y*p3.y,
        p4.x, p4.y, 1., 0.,  0.,  0., -np4.x*p4.x, -np4.x*p4.y,
        0.,  0.,  0., p4.x, p4.y, 1., -np4.y*p4.x, -np4.y*p4.y],
    );
    let axp = SVector::from_row_slice(&[np1.x, np1.y, np2.x, np2.y, np3.x, np3.y, np4.x, np4.y]);
    let decomp = a.lu();
    let p = decomp.solve(&axp)?;
    let transformation = Matrix3::new(
        p[0], p[1], p[2],
        p[3], p[4], p[5],
        p[6], p[7], 1.
    );
    Some(transformation.transform_point(&aim_point))
}

fn transform_aim_point_to_identity(center_aim: Point2<f64>, p1: Point2<f64>, p2: Point2<f64>, p3: Point2<f64>, p4: Point2<f64>) -> Option<Point2<f64>> {
    transform_aim_point(center_aim, p1, p2, p3, p4,
                        Point2::new(0.5, 1.), Point2::new(0., 0.5),
                        Point2::new(0.5, 0.), Point2::new(1., 0.5))
}

fn sort_clockwise(a: &mut [Point2<f64>]) {
    if a[0].y < a[1].y { a.swap(0, 1); }
    if a[2].y > a[3].y { a.swap(2, 3); }
    if a[0].y < a[3].y { a.swap(0, 3); }
    if a[2].y > a[1].y { a.swap(2, 1); }
    if a[1].x > a[3].x { a.swap(1, 3); }
}

pub struct MotRunner {
    pub state: Arc<Mutex<MotState>>,
    pub device: Option<UsbDevice>,
}

impl MotRunner {
    pub async fn run(&self) {
        loop {
            if self.device.is_none() {
                return;
            }
            let device = self.device.c().unwrap();
            if let Ok((nf_data, wf_data)) = device.get_frame().await {
                let nf_data = ArrayVec::<MotData,16>::from_iter(nf_data.into_iter().filter(|x| x.area > 0));
                let wf_data = ArrayVec::<MotData,16>::from_iter(wf_data.into_iter().filter(|x| x.area > 0));
                
                let mut nf_points = ArrayVec::<Point2<f64>,16>::from_iter(nf_data.as_ref().into_iter().map(|x| Point2::new(x.cx as f64, x.cy as f64)));
                let mut wf_points = ArrayVec::<Point2<f64>,16>::from_iter(wf_data.as_ref().into_iter().map(|x| Point2::new(x.cx as f64, x.cy as f64)));

                let mut fv_aim_point = None::<Point2<f64>>;
                let mut nf_aim_point = None::<Point2<f64>>;
                let mut wf_aim_point = None::<Point2<f64>>;

                let center_aim = Point2::new(2048.0, 2048.0);

                let nf_horiz_offset = 230.;
                let nf_vert_offset = 60.;
                let nf_mag_ratio = 3.0;
                for point in &mut nf_points {
                    point.x -= nf_horiz_offset;
                    point.y -= nf_vert_offset;
                    point.x /= nf_mag_ratio;
                    point.y /= nf_mag_ratio;
                    point.x = 4095./nf_mag_ratio+point.x;
                    point.y = 4095./nf_mag_ratio+point.y;
                }

                for (points, aim_point) in [(&mut nf_points, &mut nf_aim_point), (&mut wf_points, &mut wf_aim_point)] {
                    if points.len() > 3 {
                        sort_clockwise(&mut points[0..4]);
                        *aim_point = transform_aim_point_to_identity(center_aim, points[0], points[1], points[2], points[3]);
                        if aim_point.is_none() { println!("Linear resolution failed with points: ({},{}), ({},{}), ({},{}), ({},{})", points[0].x, points[0].y, points[1].x, points[1].y, points[2].x, points[2].y, points[3].x, points[3].y); };
                    }
                }

                if nf_points.len() > 3 {
                    fv_aim_point = nf_aim_point;
                } else if nf_points.len() == 0 {
                    fv_aim_point = wf_aim_point;
                } else if wf_points.len() > 3 {
                    let mut index_claim_map = [None::<usize>; 4];
                    let mut dist_map = [999.; 4];
                    for (i, wf_point) in wf_points[0..4].iter().enumerate() {
                        for (j, nf_point) in nf_points.iter().enumerate() {
                            let _dist = ((wf_point.x-nf_point.x).powi(2) + (wf_point.y-nf_point.y).powi(2)).sqrt();
                            if dist_map[i] > _dist {
                                if index_claim_map[j].is_none() || dist_map[index_claim_map[j].unwrap()] > _dist {
                                    index_claim_map[j] = Some(i);
                                    dist_map[i] = _dist;
                                }
                            }
                        }
                    }
                    let mut nf_for_wf_map = [None::<Point2<f64>>; 4];
                    for (nf_index, wf_index) in index_claim_map.iter().enumerate() {
                        if let Some(wf_index) = wf_index {
                            nf_for_wf_map[*wf_index] = Some(nf_points[nf_index]);
                        }
                    }
                    let mut points = nf_for_wf_map.into_iter().zip(wf_points).map(
                        |(nf_point_option, wf_point)| {
                            if let Some(nf_point) = nf_point_option {
                                nf_point
                            } else {
                                wf_point
                            }
                        }
                    ).collect::<ArrayVec<Point2<f64>, 4>>();
                    fv_aim_point = transform_aim_point_to_identity(center_aim, points[0], points[1], points[2], points[3]);
                }

                let mut state = self.state.lock().await;
                state.fv_aim_point = fv_aim_point;
                state.nf_aim_point = nf_aim_point;
                state.wf_aim_point = wf_aim_point;
                state.nf_data = Some(nf_data);
                state.wf_data = Some(wf_data);
            }
            sleep(Duration::from_millis(5)).await;
        }
    }
}
