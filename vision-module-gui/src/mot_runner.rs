use std::sync::Arc;
use ats_cv::kalman::Pva2d;
use opencv_ros_camera::RosOpenCvIntrinsics;
use parking_lot::Mutex;
use std::time::Duration;
use arrayvec::ArrayVec;
use iui::concurrent::Context;
use leptos_reactive::RwSignal;
use nalgebra::{Const, Isometry3, Matrix3, Point2, Rotation3, Scalar, Translation3, Vector2, Vector3};
use sqpnp::types::{SQPSolution, SolverParameters};
use tokio::time::sleep;
use tokio_stream::StreamExt;
use tracing::{debug, info};
use crate::{CloneButShorter, Frame, MotState};
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
        euler_loop(runner.clone()),
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

            let state = &mut runner.state;
            state.nf_data = Some(nf_data);
            state.wf_data = Some(wf_data);
        }
        sleep(Duration::from_millis(5)).await;
    }
}

async fn combined_markers_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().device.c().unwrap();
    let mut combined_markers_stream = device.stream_combined_markers().await.unwrap();

    while runner.lock().device.is_some() {
        if let Some(combined_markers_report) = combined_markers_stream.next().await {
            let CombinedMarkersReport { timestamp, nf_points, wf_points, nf_radii, wf_radii } = combined_markers_report;
            debug!("timestamp: {}", timestamp);

            let mut runner = runner.lock();
            let filtered_nf_point_tuples = filter_and_create_point_id_tuples(&nf_points, &nf_radii);
            let filtered_wf_point_tuples = filter_and_create_point_id_tuples(&wf_points, &wf_radii);

            let filtered_nf_points_slice = filtered_nf_point_tuples.iter().map(|(_, p)| *p).collect::<Vec<_>>();
            let filtered_wf_points_slice = filtered_wf_point_tuples.iter().map(|(_, p)| *p).collect::<Vec<_>>();

            let mut nf_points_transformed = transform_points(&filtered_nf_points_slice, &runner.general_config.camera_model_nf);
            let mut wf_points_transformed = transform_points(&filtered_wf_points_slice, &runner.general_config.camera_model_wf);

            // let nf_point_tuples_transformed = filtered_nf_point_tuples.iter().map(|(id, _)| *id).zip(&mut nf_points_transformed).collect::<Vec<_>>();
            // let wf_point_tuples_transformed = filtered_wf_point_tuples.iter().map(|(id, _)| *id).zip(&mut wf_points_transformed).collect::<Vec<_>>();

            // fn update_positions(pva2ds: &mut [Pva2d<f64>], points: Vec<(usize, &mut Point2<f64>)>) {
            //     for (i, point) in points {
            //         pva2ds[i].step();
            //         pva2ds[i].observe(point.coords.as_ref(), &[100.0, 100.0]);
            //         point.x = pva2ds[i].position()[0];
            //         point.y = pva2ds[i].position()[1];
            //     }
            // }

            // update_positions(&mut runner.state.nf_pva2ds, nf_point_tuples_transformed);
            // update_positions(&mut runner.state.wf_pva2ds, wf_point_tuples_transformed);

            let (rotation, translation, fv_aim_point) = ats_cv::get_pose_and_aimpoint_foveated(
                &nf_points_transformed,
                &wf_points_transformed,
                runner.state.orientation,
                runner.state.screen_id,
                &ats_cv::ros_opencv_intrinsics_type_convert(&runner.general_config.camera_model_nf),
                &ats_cv::ros_opencv_intrinsics_type_convert(&runner.general_config.camera_model_wf),
                runner.general_config.stereo_iso.cast(),
            );
            if let Some(rotation) = rotation {
                runner.state.rotation_mat = rotation;
            }
            if let Some(translation) = translation {
                runner.state.translation_mat = translation;
            }
            if let Some(fv_aim_point) = fv_aim_point {
                runner.state.fv_aim_point = fv_aim_point;
            }

            if let Some(x) = calculate_individual_aim_point(&nf_points_transformed, runner.state.orientation, None, &runner.general_config.camera_model_nf) {
                runner.state.nf_aim_point = x;
            }

            if let Some(x) = calculate_individual_aim_point(&wf_points_transformed, runner.state.orientation, Some(&runner.general_config.stereo_iso.cast()), &runner.general_config.camera_model_wf) {
                runner.state.wf_aim_point = x;
            }

            let (_, wf_markers) = ats_cv::foveated::identify_markers(&[], &wf_points_transformed);
            let (_, nf_markers) = ats_cv::foveated::identify_markers(&[], &nf_points_transformed);
            let nf_markers: ArrayVec<_, 16> = nf_markers.into_iter().flatten().collect();
            let wf_markers: ArrayVec<_, 16> = wf_markers.into_iter().flatten().collect();

            runner.state.nf_points = nf_points_transformed.into_iter().filter(|p| !nf_markers.contains(&p)).collect();
            runner.state.wf_points = wf_points_transformed.into_iter().filter(|p| !wf_markers.contains(&p)).collect();
            runner.state.nf_markers = nf_markers;
            runner.state.wf_markers = wf_markers;

            let index = runner.state.nf_aim_point_history_index;
            runner.state.nf_aim_point_history[index] = runner.state.nf_aim_point;
            runner.state.nf_aim_point_history_index = (index + 1) % runner.state.nf_aim_point_history.len();
        }
    }
}

fn calculate_individual_aim_point(points: &[Point2<f64>], orientation: Rotation3<f64>, iso: Option<&Isometry3<f32>>, intrinsics: &RosOpenCvIntrinsics<f32>) -> Option<Point2<f64>> {
    let fx = intrinsics.p.m11 * (4095./98.);
    let fy = intrinsics.p.m22 * (4095./98.);

    let gravity_vec = orientation.inverse_transform_vector(&Vector3::z());
	let gravity_angle = f64::atan2(-gravity_vec.z as f64, -gravity_vec.x as f64) + std::f64::consts::PI/2.;

    if points.len() > 3 {
        let mut rotated_points = ats_cv::mot_rotate(&points, -gravity_angle);
        sort_points(&mut rotated_points, MarkerPattern::Rectangle);
        // todo rotating back is bad, select with slice instead
        let points = ats_cv::mot_rotate(&rotated_points, gravity_angle);

        let projections = ats_cv::calculate_projections(
            &points,
            // 1/math.tan(38.3 / 180 * math.pi / 2) * 2047.5 (value used in the sim)
            Vector2::new(fx as f64, fy as f64),
            Vector2::new(4095., 4095.),
        );
        let solution = ats_cv::solve_pnp_with_dynamic_screen_points(
            projections.as_slice(),
            &[
                Point2::new(0.35, 0.),
                Point2::new(0.65, 0.),
                Point2::new(0.65, 1.),
                Point2::new(0.35, 1.),
            ],
            16./9.,
            1.,
        );
        if let Some(sol) = solution {
            let r_hat = Rotation3::from_matrix_unchecked(sol.r_hat.reshape_generic(Const::<3>, Const::<3>).transpose());
            let t = Translation3::from(sol.t);
            let tf = t * r_hat;
            let ctf = tf.inverse();

            let flip_yz = Matrix3::new(
                1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, -1.0,
            );

            if let Some(iso) = iso {
                let iso = iso.cast();
                let rotation_mat = flip_yz * (iso.rotation * ctf.rotation).to_rotation_matrix() * flip_yz;
                let translation_mat = flip_yz * ctf.translation.vector;

                let screen_3dpoints = ats_cv::calculate_screen_3dpoints(1., 16./9.);

                return ats_cv::calculate_aimpoint_from_pose_and_screen_3dpoints(&rotation_mat, &translation_mat, &screen_3dpoints);
            } else {
                let rotation_mat = flip_yz * ctf.rotation.matrix() * flip_yz;
                let translation_mat = flip_yz * ctf.translation.vector;

                let screen_3dpoints = ats_cv::calculate_screen_3dpoints(1., 16./9.);

                return ats_cv::calculate_aimpoint_from_pose_and_screen_3dpoints(&rotation_mat, &translation_mat, &screen_3dpoints);
            }
        }
    }
    None
}

fn filter_and_create_point_id_tuples(points: &[Point2<u16>], radii: &[u8]) -> Vec<(usize, Point2<f64>)> {
    points.iter().zip(radii.iter())
        .enumerate()
        .filter_map(|(id, (pos, &r))| if r > 0 { Some((id, Point2::new(pos.x as f64, pos.y as f64))) } else { None })
        .collect()
}

fn transform_points(points: &[Point2<f64>], camera_intrinsics: &RosOpenCvIntrinsics<f32>) -> Vec<Point2<f64>> {
    let scaled_points = points.iter().map(|p| Point2::new(p.x / 4095. * 98., p.y / 4095. * 98.)).collect::<Vec<_>>();
    let undistorted_points = ats_cv::undistort_points(&ats_cv::ros_opencv_intrinsics_type_convert(camera_intrinsics), &scaled_points);
    undistorted_points.iter().map(|p| Point2::new(p.x / 98. * 4095., p.y / 98. * 4095.)).collect()
}

async fn accel_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().device.c().unwrap();
    let mut accel_stream = device.stream_accel().await.unwrap();
    while runner.lock().device.is_some() {
        if let Some(accel) = accel_stream.next().await {
            let mut runner = runner.lock();
            // runner.state.filter.predict(accel.accel, accel.gyro, 1/runner.general_config.accel_odr);
        }
    }
}

async fn euler_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().device.c().unwrap();
    let mut euler_stream = device.stream_euler_angles().await.unwrap();
    while runner.lock().device.is_some() {
        if let Some(euler_angles) = euler_stream.next().await {
            let mut runner = runner.lock();
            runner.state.orientation = euler_angles.rotation;
        }
    }
}

// todo use an aimpoint history to choose the aimpoint closest to the timestamp
async fn impact_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().device.c().unwrap();
    let mut impact_stream = device.stream_impact().await.unwrap();
    while runner.lock().device.is_some() {
        if let Some(_impact) = impact_stream.next().await {
            let runner = runner.lock();
            if runner.record_impact {
                let mut frame = Frame {
                    nf_aim_point_x: None,
                    nf_aim_point_y: None,
                };

                {
                    let nf_aim_point = runner.state.nf_aim_point_history[runner.state.nf_aim_point_history_index];
                    let nf_aim_point = nf_aim_point + runner.nf_offset;
                    frame.nf_aim_point_x = Some(nf_aim_point.x);
                    frame.nf_aim_point_y = Some(nf_aim_point.y);
                }

                if runner.datapoints.is_locked() {
                    continue;
                }

                runner.datapoints.lock().push(frame);

                let ui_update = runner.ui_update.c();

                runner.ui_ctx.queue_main(move || {
                    leptos_reactive::SignalSet::set(&ui_update, ());
                });
            }
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
