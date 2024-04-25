use std::sync::Arc;
use opencv_ros_camera::RosOpenCvIntrinsics;
use parking_lot::Mutex;
use std::time::Duration;
use ahrs::Ahrs;
use arrayvec::ArrayVec;
use ats_cv::get_perspective_transform;
use iui::concurrent::Context;
use leptos_reactive::RwSignal;
use nalgebra::{Const, Matrix, Matrix1xX, Matrix2xX, Matrix3, MatrixXx1, MatrixXx2, Point2, Rotation3, Scalar, Translation3, Vector2, Vector3};
use sqpnp::types::{SQPSolution, SolverParameters};
use tokio::time::sleep;
use tokio_stream::StreamExt;
use tracing::{debug, info};
use crate::{CloneButShorter, MotState};
use crate::marker_config_window::MarkersSettings;
use ats_usb::device::UsbDevice;
use ats_usb::packet::{CombinedMarkersReport, GeneralConfig, MarkerPattern, MotData};

pub fn transform_aim_point_to_identity(center_aim: Point2<f64>, p1: Point2<f64>, p2: Point2<f64>, p3: Point2<f64>, p4: Point2<f64>) -> Option<Point2<f64>> {
    ats_cv::transform_aim_point(center_aim, p1, p2, p3, p4,
                        Point2::new(0.5, 1.), Point2::new(0., 0.5),
                        Point2::new(0.5, 0.), Point2::new(1., 0.5))
}

pub fn my_pnp(projections: &[Vector2<f64>]) -> Option<SQPSolution> {
    let _3dpoints = [
        Vector3::new((0.35 - 0.5) * 16./9., -0.5, 0.),
        Vector3::new((0.65 - 0.5) * 16./9., -0.5, 0.),
        Vector3::new((0.65 - 0.5) * 16./9., 0.5, 0.),
        Vector3::new((0.35 - 0.5) * 16./9., 0.5, 0.),
    ];
    let solver = sqpnp::PnpSolver::new(&_3dpoints, &projections, None, SolverParameters::default());
    if let Some(mut solver) = solver {
        solver.solve();
        debug!("pnp found {} solutions", solver.number_of_solutions());
        if solver.number_of_solutions() >= 1 {
            return Some(solver.solution_ptr(0).unwrap().clone());
        }
    } else {
        info!("pnp solver failed");
    }
    None
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
    tokio::join!(
        combined_markers_loop(runner.clone()),
        accel_loop(runner.clone()),
        impact_loop(runner.clone()),
    );
}

pub async fn frame_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().device.c().unwrap();
    let mut mot_data_stream = device.stream_mot_data().await.unwrap();
    loop {
        if runner.lock().device.is_none() {
            return;
        }
        // if let Ok((nf_data, wf_data)) = device.get_frame().await {
        if let Some(mot_data) = mot_data_stream.next().await {
            let nf_data = mot_data.mot_data_nf;
            let wf_data = mot_data.mot_data_wf;
            let mut runner = runner.lock();
            let nf_data = ArrayVec::<MotData,16>::from_iter(nf_data.into_iter().filter(|x| x.area > 0));
            // let nf_data = ArrayVec::<MotData,16>::from_iter(dummy_nf_data());
            let wf_data = ArrayVec::<MotData,16>::from_iter(wf_data.into_iter().filter(|x| x.area > 0));

            let mut nf_points = ArrayVec::<Point2<f64>,16>::from_iter(nf_data.as_ref().into_iter().map(|x| Point2::new(x.cx as f64, x.cy as f64)));
            let _wf_points = ArrayVec::<Point2<f64>,16>::from_iter(wf_data.as_ref().into_iter().map(|x| Point2::new(x.cx as f64, x.cy as f64)));

            let _fv_aim_point = None::<Point2<f64>>;
            let _nf_aim_point = None::<Point2<f64>>;
            let _wf_aim_point = None::<Point2<f64>>;

            let _center_aim = Point2::new(2048.0, 2048.0);

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
        }
        sleep(Duration::from_millis(5)).await;
    }
}

fn ray_plane_intersection(ray_origin: Vector3<f64>, ray_direction: Vector3<f64>, plane_normal: Vector3<f64>, plane_point: Vector3<f64>) -> Vector3<f64> {
    let d = plane_normal.dot(&plane_point);
    let t = (d - plane_normal.dot(&ray_origin)) / plane_normal.dot(&ray_direction);
    ray_origin + t * ray_direction
}

async fn combined_markers_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().device.c().unwrap();
    let mut combined_markers_stream = device.stream_combined_markers().await.unwrap();
    while runner.lock().device.is_some() {
        if let Some(combined_markers_report) = combined_markers_stream.next().await {
            let CombinedMarkersReport { timestamp, nf_points, wf_points, nf_radii, wf_radii } = combined_markers_report;
            debug!("timestamp: {}", timestamp);

            let mut runner = runner.lock();
            debug!("nf_positions: {:?}, wf_positions: {:?}, nf_radii: {:?}, wf_radii: {:?}", nf_points, wf_points, nf_radii, wf_radii);

            let nf_points = nf_points.into_iter().zip(nf_radii.into_iter()).filter_map(|(pos, r)| if r > 0 { Some(pos) } else { None }).collect::<Vec<_>>();
            let nf_points = nf_points.into_iter().map(|pos| Point2::new(pos.x as f64, pos.y as f64)).collect::<Vec<_>>();

            let wf_points = wf_points.into_iter().zip(wf_radii.into_iter()).filter_map(|(pos, r)| if r > 0 { Some(pos) } else { None }).collect::<Vec<_>>();
            let wf_points = wf_points.into_iter().map(|pos| Point2::new(pos.x as f64, pos.y as f64)).collect::<Vec<_>>();

            let mut nf_points = ats_cv::undistort_points(&runner.state.camera_model_nf, &nf_points);

            let mut wf_points = ats_cv::undistort_points(&runner.state.camera_model_wf, &wf_points);

            if nf_points.len() > 3 {
                let state = &mut runner.state;
                for (i, pva2d) in state.nf_pva2ds.iter_mut().enumerate() {
                    pva2d.step();
                    pva2d.observe(nf_points[i].coords.as_ref(), &[100.0, 100.0]);
                    nf_points[i].x = pva2d.position()[0];
                    nf_points[i].y = pva2d.position()[1];
                }
            }

            if wf_points.len() > 3 {
                let state = &mut runner.state;
                for (i, pva2d) in state.wf_pva2ds.iter_mut().enumerate() {
                    pva2d.step();
                    pva2d.observe(wf_points[i].coords.as_ref(), &[100.0, 100.0]);
                    wf_points[i].x = pva2d.position()[0];
                    wf_points[i].y = pva2d.position()[1];
                }
            }

            let gravity: Vector3<f64> = runner.state.orientation.quat.inverse_transform_vector(&Vector3::z()).into();
            let gravity_angle = -gravity.x.atan2(gravity.z);

            let rotated_wf = ats_cv::mot_rotate(&wf_points, -gravity_angle);
            match ats_cv::calculate_screen_id(&rotated_wf) {
                Some(id) => {
                    runner.state.screen_id = id;
                },
                None => {},
            }
            let (rotation, translation, fv_aim_point) = ats_cv::get_pose_and_aimpoint(&nf_points, &wf_points, gravity_angle, runner.state.screen_id);
            if let Some(rotation) = rotation {
                runner.state.rotation_mat = rotation;
            }
            if let Some(translation) = translation {
                runner.state.translation_mat = translation;
            }

            let nf_aim_point = if nf_points.len() > 3 {
                let mut nf_positions = nf_points.clone();
                sort_points(&mut nf_positions, MarkerPattern::Rectangle);
                let projections = nf_positions.iter().map(|pos| {
                    // 1/math.tan(38.3 / 180 * math.pi / 2) * 2047.5 (value used in the sim)
                    let f = 5896.181431117499;
                    let x = (pos.x - 2047.5) / f;
                    let y = (pos.y - 2047.5) / f;
                    Vector2::new(x, y)
                }).collect::<Vec<Vector2<_>>>();
                let solution = my_pnp(&projections);
                if let Some(solution) = solution {
                    let r_hat = Rotation3::from_matrix_unchecked(solution.r_hat.reshape_generic(Const::<3>, Const::<3>).transpose());
                    let t = Translation3::from(solution.t);
                    let tf = t * r_hat;
                    let ctf = tf.inverse();

                    let flip_yz = Matrix3::new(
                        1., 0., 0.,
                        0., -1., 0.,
                        0., 0., -1.,
                    );
                    runner.state.rotation_mat = flip_yz * ctf.rotation.matrix() * flip_yz;
                    runner.state.translation_mat = flip_yz * ctf.translation.vector;

                    let screen_3dpoints = [
                        Vector3::new((0.0 - 0.5) * 16./9., -0.5, 0.),
                        Vector3::new((1.0 - 0.5) * 16./9., -0.5, 0.),
                        Vector3::new((1.0 - 0.5) * 16./9., 0.5, 0.),
                        Vector3::new((0.0 - 0.5) * 16./9., 0.5, 0.),
                    ];

                    let ray_origin = runner.state.translation_mat;
                    let ray_direction = runner.state.rotation_mat * Vector3::new(0., 0., 1.);
                    let plane_normal = (screen_3dpoints[1] - screen_3dpoints[0]).cross(&(screen_3dpoints[2] - screen_3dpoints[0]));
                    let plane_point = screen_3dpoints[0];
                    let aim_point = ray_plane_intersection(ray_origin, ray_direction, plane_normal, plane_point);

                    let aim_point = Point2::new(
                        (aim_point.x - screen_3dpoints[0].x) / (screen_3dpoints[2].x - screen_3dpoints[0].x),
                        (aim_point.y - screen_3dpoints[0].y) / (screen_3dpoints[2].y - screen_3dpoints[0].y),
                    );

                    let aim_point = Point2::new(aim_point.x, 1. - aim_point.y);
                    Some(aim_point)
                } else {
                    None
                }
            } else {
                None
            };

            let wf_aim_point = if wf_points.len() > 3 {
                let mut wf_positions = wf_points.clone();
                sort_points(&mut wf_positions, MarkerPattern::Rectangle);
                let center_aim = Point2::new(2048.0, 2048.0);
                let projections = wf_positions.iter().map(|pos| {
                    // 1/math.tan(111.3 / 180 * math.pi / 2) * 2047.5 (value used in the sim)
                    let f = 1399.329592256857;
                    let x = (pos.x - 2047.5) / f;
                    let y = (pos.y - 2047.5) / f;
                    Vector2::new(x, y)
                }).collect::<Vec<Vector2<_>>>();
                let solution = my_pnp(&projections);
                if let Some(solution) = solution {
                    let r_hat = Rotation3::from_matrix_unchecked(solution.r_hat.reshape_generic(Const::<3>, Const::<3>).transpose());
                    let t = Translation3::from(solution.t);
                    let tf = t * r_hat;
                    let ctf = tf.inverse();

                    let flip_yz = Matrix3::new(
                        1., 0., 0.,
                        0., -1., 0.,
                        0., 0., -1.,
                    );
                    let rotation_mat = flip_yz * ctf.rotation.matrix() * flip_yz;
                    let translation_mat = flip_yz * ctf.translation.vector;

                    let screen_3dpoints = [
                        Vector3::new((0.0 - 0.5) * 16./9., -0.5, 0.),
                        Vector3::new((1.0 - 0.5) * 16./9., -0.5, 0.),
                        Vector3::new((1.0 - 0.5) * 16./9., 0.5, 0.),
                        Vector3::new((0.0 - 0.5) * 16./9., 0.5, 0.),
                    ];

                    let ray_origin = translation_mat;
                    let ray_direction = rotation_mat * Vector3::new(0., 0., 1.);
                    let plane_normal = (screen_3dpoints[1] - screen_3dpoints[0]).cross(&(screen_3dpoints[2] - screen_3dpoints[0]));
                    let plane_point = screen_3dpoints[0];
                    let aim_point = ray_plane_intersection(ray_origin, ray_direction, plane_normal, plane_point);

                    let aim_point = Point2::new(
                        (aim_point.x - screen_3dpoints[0].x) / (screen_3dpoints[2].x - screen_3dpoints[0].x),
                        (aim_point.y - screen_3dpoints[0].y) / (screen_3dpoints[2].y - screen_3dpoints[0].y),
                    );

                    if nf_aim_point == None {
                        runner.state.translation_mat = translation_mat;
                        runner.state.rotation_mat = rotation_mat;
                    }

                    let aim_point = Point2::new(aim_point.x, 1. - aim_point.y);
                    Some(aim_point)
                } else {
                    None
                }
            } else {
                None
            };
            runner.state.nf_points = nf_points.into_iter().collect::<ArrayVec<Point2<f64>, 16>>();
            runner.state.wf_points = wf_points.into_iter().collect::<ArrayVec<Point2<f64>, 16>>();
            runner.state.nf_aim_point = nf_aim_point;
            runner.state.wf_aim_point = wf_aim_point;
            runner.state.fv_aim_point = fv_aim_point;
        }
    }
}

async fn accel_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().device.c().unwrap();
    let mut accel_stream = device.stream_accel().await.unwrap();
    while runner.lock().device.is_some() {
        if let Some(accel) = accel_stream.next().await {
            let mut runner = runner.lock();
            debug!("accel: {:?}", accel);
            let _ = runner.state.orientation.update_imu(&Vector3::from(accel.gyro), &Vector3::from(accel.accel));
        }
    }
}

async fn impact_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().device.c().unwrap();
    let mut frame_stream = device.stream_impact().await.unwrap();
    while runner.lock().device.is_some() {
        if let Some(()) = frame_stream.next().await {
            // todo
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
