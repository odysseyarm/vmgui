use std::sync::Arc;
use std::time::Duration;
use nalgebra::{Matrix3, Point2, SMatrix, SVector};
use tokio::sync::Mutex;
use tokio::time::sleep;
use crate::{CloneButShorter, MotState};
use crate::device::UsbDevice;

fn transform_aim_point(aim_point: Point2<f64>, p1: Point2<f64>, p2: Point2<f64>, p3: Point2<f64>, p4: Point2<f64>,
             np1: Point2<f64>, np2: Point2<f64>, np3: Point2<f64>, np4: Point2<f64>) -> Point2<f64> {

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
    let p = decomp.solve(&axp).expect("Linear resolution failed.");
    let transformation = Matrix3::new(
        p[0], p[1], p[2],
        p[3], p[4], p[5],
        p[6], p[7], 1.
    );
    transformation.transform_point(&aim_point)
}

fn transform_aim_point_to_identity(aim_point: Point2<f64>, p1: Point2<f64>, p2: Point2<f64>, p3: Point2<f64>, p4: Point2<f64>) -> Point2<f64> {
    transform_aim_point(aim_point, p1, p2, p3, p4,
                        Point2::new(0.5, 0.), Point2::new(1., 0.5),
                        Point2::new(0.5, 1.), Point2::new(0., 0.5))
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
            let (nf_data, wf_data) = device.get_frame().await.expect("Problem getting frame from device");

            let mut nf_aim_point = None::<Point2<f64>>;
            if nf_data[3].area > 0 {
                nf_aim_point = Some(transform_aim_point_to_identity(Point2::new(2048.0,2048.0),
                                              Point2::new(nf_data[0].cx as f64, nf_data[0].cy as f64),
                                              Point2::new(nf_data[1].cx as f64, nf_data[1].cy as f64),
                                              Point2::new(nf_data[2].cx as f64, nf_data[2].cy as f64),
                                              Point2::new(nf_data[3].cx as f64, nf_data[3].cy as f64)));
            }

            let mut wf_aim_point = None::<Point2<f64>>;
            if wf_data[3].area > 0 {
                wf_aim_point = Some(transform_aim_point_to_identity(Point2::new(2048.0,2048.0),
                                                Point2::new(wf_data[0].cx as f64, wf_data[0].cy as f64),
                                                Point2::new(wf_data[1].cx as f64, wf_data[1].cy as f64),
                                                Point2::new(wf_data[2].cx as f64, wf_data[2].cy as f64),
                                                Point2::new(wf_data[3].cx as f64, wf_data[3].cy as f64)));
            }

            let mut state = self.state.lock().await;
            state.nf_aim_point = nf_aim_point;
            state.wf_aim_point = wf_aim_point;
            state.nf_data = Some(nf_data);
            state.wf_data = Some(wf_data);
            sleep(Duration::from_millis(5)).await;
        }
    }
}
