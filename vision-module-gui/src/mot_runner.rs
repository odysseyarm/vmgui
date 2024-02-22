use std::sync::Arc;
use std::time::Duration;
use arrayvec::ArrayVec;
use ats_cv::get_perspective_transform;
use iui::concurrent::Context;
use leptos_reactive::RwSignal;
use nalgebra::{Matrix2x4, Point2, Scalar, Vector2};
use tokio::sync::Mutex;
use tokio::time::sleep;
use tokio_stream::StreamExt;
use tracing::debug;
use crate::{CloneButShorter, MotState};
use crate::device::UsbDevice;
use crate::marker_config_window::MarkersSettings;
use crate::packet::{AimPointReport, GeneralConfig, MarkerPattern, MotData};

pub fn transform_aim_point_to_identity(center_aim: Point2<f64>, p1: Point2<f64>, p2: Point2<f64>, p3: Point2<f64>, p4: Point2<f64>) -> Option<Point2<f64>> {
    ats_cv::transform_aim_point(center_aim, p1, p2, p3, p4,
                        Point2::new(0.5, 1.), Point2::new(0., 0.5),
                        Point2::new(0.5, 0.), Point2::new(1., 0.5))
}

/// Given 4 points in the following shape
///
/// ```
/// +--x
/// |
/// y                 top
///
///
///   left                              right
///
///
///                  bottom
/// ```
///
/// Sort them into the order bottom, left, top, right
pub fn sort_diamond<T: Scalar + PartialOrd>(a: &mut [Point2<T>]) {
    if a[0].y < a[1].y { a.swap(0, 1); }
    if a[2].y > a[3].y { a.swap(2, 3); }
    if a[0].y < a[3].y { a.swap(0, 3); }
    if a[2].y > a[1].y { a.swap(2, 1); }
    if a[1].x > a[3].x { a.swap(1, 3); }
}

/// Given 4 points in the following shape
///
/// ```
/// +--x
/// |
/// y        a              b
///
///
///          c              d
/// ```
///
/// Sort them into the order a, b, d, c
pub fn sort_rectangle<T: Scalar + PartialOrd>(a: &mut [Point2<T>]) {
    if a[0].y > a[2].y { a.swap(0, 2); }
    if a[1].y > a[3].y { a.swap(1, 3); }
    if a[0].y > a[1].y { a.swap(0, 1); }
    if a[2].y > a[3].y { a.swap(2, 3); }
    if a[1].y > a[2].y { a.swap(1, 2); }
    if a[0].x > a[1].x { a.swap(0, 1); }
    if a[2].x < a[3].x { a.swap(2, 3); }
}

pub fn sort_points<T: Scalar + PartialOrd>(a: &mut [Point2<T>], pattern: MarkerPattern) {
    match pattern {
        MarkerPattern::Diamond => sort_diamond(a),
        MarkerPattern::Rectangle => sort_rectangle(a),
    }
}

pub struct MotRunner {
    pub state: MotState,
    pub device: Option<UsbDevice>,
    pub markers_settings: MarkersSettings,
    pub general_config: GeneralConfig,
    pub record_impact: bool,
    pub datapoints: Arc<Mutex<Vec<crate::Frame>>>,
    pub ui_update: RwSignal<()>,
    pub ui_ctx: Context,
    pub nf_offset: Vector2<f64>,
}

pub async fn run(runner: Arc<Mutex<MotRunner>>) {
    // doesn't work too lazy to figure out why
    // let halt = Cell::new(false);
    tokio::join!(
        // frame_loop(&halt, runner.clone()),
        // impact_loop(&halt, runner.clone()),
        frame_loop(runner.clone()),
        impact_loop(runner.clone()),
        aim_loop(runner.clone()),
    );
}

async fn frame_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().await.device.c().unwrap();
    let mut mot_data_stream = device.stream_mot_data().await.unwrap();
    loop {
        if runner.lock().await.device.is_none() {
            return;
        }
        // if let Ok((nf_data, wf_data)) = device.get_frame().await {
        if let Some(mot_data) = mot_data_stream.next().await {
            let nf_data = mot_data.mot_data_nf;
            let wf_data = mot_data.mot_data_wf;
            let mut runner = runner.lock().await;
            let nf_data = ArrayVec::<MotData,16>::from_iter(nf_data.into_iter().filter(|x| x.area > 0));
            // let nf_data = ArrayVec::<MotData,16>::from_iter(dummy_nf_data());
            let wf_data = ArrayVec::<MotData,16>::from_iter(wf_data.into_iter().filter(|x| x.area > 0));

            let mut nf_points = ArrayVec::<Point2<f64>,16>::from_iter(nf_data.as_ref().into_iter().map(|x| Point2::new(x.cx as f64, x.cy as f64)));
            let _wf_points = ArrayVec::<Point2<f64>,16>::from_iter(wf_data.as_ref().into_iter().map(|x| Point2::new(x.cx as f64, x.cy as f64)));

            let _fv_aim_point = None::<Point2<f64>>;
            let _nf_aim_point = None::<Point2<f64>>;
            let _wf_aim_point = None::<Point2<f64>>;

            let _center_aim = Point2::new(2048.0, 2048.0);

            // kal man for nf rn
            if nf_points.len() > 3 {
                let state = &mut runner.state;
                for (i, pva2d) in state.nf_pva2ds.iter_mut().enumerate() {
                    pva2d.step();
                    pva2d.observe(nf_points[i].coords.as_ref(), &[100.0, 100.0]);
                    nf_points[i].x = pva2d.position()[0];
                    nf_points[i].y = pva2d.position()[1];
                }
            }

            // for (points, aim_point) in [(&mut nf_points, &mut nf_aim_point), (&mut wf_points, &mut wf_aim_point)] {
            //     if points.len() > 3 {
            //         sort_clockwise(&mut points[0..4]);
            //         let top = runner.markers_settings.views[0].marker_top.position;
            //         let left = runner.markers_settings.views[0].marker_left.position;
            //         let bottom = runner.markers_settings.views[0].marker_bottom.position;
            //         let right = runner.markers_settings.views[0].marker_right.position;
            //         *aim_point = ats_cv::transform_aim_point(center_aim, points[0], points[1], points[2], points[3],
            //                                                    Point2::new(rescale(bottom.x as f64), rescale(bottom.y as f64)), // bottom
            //                                                    Point2::new(rescale(left.x as f64), rescale(left.y as f64)), // left
            //                                                    Point2::new(rescale(top.x as f64), rescale(top.y as f64)), // top
            //                                                    Point2::new(rescale(right.x as f64), rescale(right.y as f64)), // right
            //                                                    // Point2::new(0., 0.5), // left
            //                                                    // Point2::new(0.5, 0.), // top
            //                                                    // Point2::new(1., 0.5), // right
            //         );
            //         if aim_point.is_none() {
            //             println!("Linear resolution failed with points: ({},{}), ({},{}), ({},{}), ({},{})", points[0].x, points[0].y, points[1].x, points[1].y, points[2].x, points[2].y, points[3].x, points[3].y);
            //             println!("Also: ({},{}), ({},{}), ({},{}), ({},{})", bottom.x, bottom.y, left.x, left.y, top.x, top.y, right.x, right.y);
            //         };
            //     }
            // }

            // if nf_points.len() > 3 {
            //     fv_aim_point = nf_aim_point;
            // } else if nf_points.len() == 0 {
            //     fv_aim_point = wf_aim_point;
            // } else if wf_points.len() > 3 {
            //     let mut index_claim_map = [None::<usize>; 4];
            //     let mut dist_map = [999.; 4];
            //     for (i, wf_point) in wf_points[0..4].iter().enumerate() {
            //         for (j, nf_point) in nf_points.iter().enumerate() {
            //             let _dist = ((wf_point.x-nf_point.x).powi(2) + (wf_point.y-nf_point.y).powi(2)).sqrt();
            //             if dist_map[i] > _dist {
            //                 if index_claim_map[j].is_none() || dist_map[index_claim_map[j].unwrap()] > _dist {
            //                     index_claim_map[j] = Some(i);
            //                     dist_map[i] = _dist;
            //                 }
            //             }
            //         }
            //     }
            //     let mut nf_for_wf_map = [None::<Point2<f64>>; 4];
            //     for (nf_index, wf_index) in index_claim_map.iter().enumerate() {
            //         if let Some(wf_index) = wf_index {
            //             nf_for_wf_map[*wf_index] = Some(nf_points[nf_index]);
            //         }
            //     }
            //     let mut points = nf_for_wf_map.into_iter().zip(wf_points).map(
            //         |(nf_point_option, wf_point)| {
            //             if let Some(nf_point) = nf_point_option {
            //                 nf_point
            //             } else {
            //                 wf_point
            //             }
            //         }
            //     ).collect::<ArrayVec<Point2<f64>, 4>>();
            //     let top = runner.markers_settings.views[0].marker_top.position;
            //     let left = runner.markers_settings.views[0].marker_left.position;
            //     let bottom = runner.markers_settings.views[0].marker_bottom.position;
            //     let right = runner.markers_settings.views[0].marker_right.position;
            //     fv_aim_point = ats_cv::transform_aim_point(center_aim, points[0], points[1], points[2], points[3],
            //            Point2::new(rescale(bottom.x as f64), rescale(bottom.y as f64)), // bottom
            //            Point2::new(rescale(left.x as f64), rescale(left.y as f64)), // left
            //            Point2::new(rescale(top.x as f64), rescale(top.y as f64)), // top
            //            Point2::new(rescale(right.x as f64), rescale(right.y as f64)), // right
            //            // Point2::new(0., 0.5), // left
            //            // Point2::new(0.5, 0.), // top
            //            // Point2::new(1., 0.5), // right
            //     );
            // }

            let state = &mut runner.state;
            // state.fv_aim_point = fv_aim_point;
            // state.nf_aim_point = nf_aim_point;
            // state.wf_aim_point = wf_aim_point;
            state.nf_data = Some(nf_data);
            state.wf_data = Some(wf_data);
            state.gravity_angle = mot_data.gravity_angle as f64;
        }
        sleep(Duration::from_millis(5)).await;
    }
}

async fn impact_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().await.device.c().unwrap();
    let mut frame_stream = device.stream_impact().await.unwrap();
    while runner.lock().await.device.is_some() {
        if let Some((_x, _y)) = frame_stream.next().await {
            let runner = runner.lock().await;
            if runner.record_impact {
                let mut datapoints = runner.datapoints.lock().await;

                let mut frame = crate::Frame {
                    nf_aim_point_x: None,
                    nf_aim_point_y: None,
                };

                let p = Point2::new(_x, _y).cast::<f64>();

                // TODO don't assume view[0]
                let view = &runner.markers_settings.views[0];
                let top = view.marker_top.position;
                let bottom = view.marker_bottom.position;
                let left = view.marker_left.position;
                let right = view.marker_right.position;

                let top = Point2::new(top.x, top.y).cast::<f64>();
                let bottom = Point2::new(bottom.x, bottom.y).cast::<f64>();
                let left = Point2::new(left.x, left.y).cast::<f64>();
                let right = Point2::new(right.x, right.y).cast::<f64>();

                // TODO maybe don't recompute this in a hot loop lol
                let [bottom_p, left_p, top_p, right_p] = runner.general_config.marker_pattern.marker_positions();
                let m = Matrix2x4::from_columns(&[
                    bottom_p.coords,
                    left_p.coords,
                    top_p.coords,
                    right_p.coords,
                ]);
                let m: Matrix2x4<f64> = m.add_scalar(-0.5) * 4094.0;
                let tf = get_perspective_transform(
                    m.column(0).xy().into(), // bottom
                    m.column(1).xy().into(), // left
                    m.column(2).xy().into(), // top
                    m.column(3).xy().into(), // right
                    bottom,
                    left,
                    top,
                    right,
                ).unwrap();
                let p = tf.transform_point(&p);

                let p = Point2::new(((p.x as f64)/2047.+1.)/2., ((p.y as f64)/2047.+1.)/2.);

                frame.nf_aim_point_x = Some(p.x);
                frame.nf_aim_point_y = Some(p.y);

                datapoints.push(frame);

                let ui_update = runner.ui_update.c();

                runner.ui_ctx.queue_main(move || {
                    leptos_reactive::SignalSet::set(&ui_update, ());
                });
            }
        }
    }
}

async fn aim_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().await.device.c().unwrap();
    let mut aim_stream = device.stream_aim().await.unwrap();
    while runner.lock().await.device.is_some() {
        if let Some(aim_report) = aim_stream.next().await {
            let AimPointReport { x, y, screen_id } = aim_report;
            let mut runner = runner.lock().await;
            debug!("aim: ({x}, {y}), screen_id: {screen_id}");
            let p = Point2::new(x, y).cast::<f64>();

            // TODO don't assume view[0]
            let view = &mut runner.markers_settings.views[0];
            let top = view.marker_top.position;
            let bottom = view.marker_bottom.position;
            let left = view.marker_left.position;
            let right = view.marker_right.position;

            let top = Point2::new(top.x, top.y).cast::<f64>();
            let bottom = Point2::new(bottom.x, bottom.y).cast::<f64>();
            let left = Point2::new(left.x, left.y).cast::<f64>();
            let right = Point2::new(right.x, right.y).cast::<f64>();

            // TODO maybe don't recompute this in a hot loop lol
            let [bottom_m, left_m, top_m, right_m] = runner.general_config.marker_pattern.marker_positions();
            let m = Matrix2x4::from_columns(&[
                bottom_m.coords,
                left_m.coords,
                top_m.coords,
                right_m.coords,
            ]);
            let m: Matrix2x4<f64> = m.add_scalar(-0.5) * 4094.0;
            let tf = get_perspective_transform(
                m.column(0).xy().into(), // bottom
                m.column(1).xy().into(), // left
                m.column(2).xy().into(), // top
                m.column(3).xy().into(), // right
                bottom,
                left,
                top,
                right,
            ).unwrap();
            let p = tf.transform_point(&p);

            let state = &mut runner.state;
            state.nf_aim_point = Some(Point2::new(((p.x as f64)/2047.+1.)/2., ((p.y as f64)/2047.+1.)/2.));
            state.screen_id = aim_report.screen_id;
        }
    }
}

pub fn rescale(val: f64) -> f64 {
    rescale_generic(-2047.0, 2047.0, 0.0, 1.0, val)
}
fn rescale_generic(lo1: f64, hi1: f64, lo2: f64, hi2: f64, val: f64) -> f64 {
    (val - lo1) / (hi1 - lo1) * (hi2 - lo2) + lo2
}

fn _dummy_nf_data() -> [MotData; 4] {
    [
        MotData {
            cx: 1047,
            cy: 2047,
            area: 1,
            ..Default::default()
        },
        MotData {
            cx: 3347,
            cy: 2047,
            area: 1,
            ..Default::default()
        },
        MotData {
            cx: 2047,
            cy: 1047,
            area: 1,
            ..Default::default()
        },
        MotData {
            cx: 2047,
            cy: 3047,
            area: 1,
            ..Default::default()
        },
    ]
}
