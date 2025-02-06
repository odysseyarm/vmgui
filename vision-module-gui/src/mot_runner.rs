use crate::{CloneButShorter, Marker, MotState, TestFrame};
use ahrs::Ahrs;
use arrayvec::ArrayVec;
use ats_cv::foveated::MARKER_PATTERN_LEN;
use ats_cv::{calculate_rotational_offset, to_normalized_image_coordinates, ScreenCalibration};
use ats_usb::device::UsbDevice;
use ats_usb::packets::vm::{CombinedMarkersReport, GeneralConfig, MotData};
use iui::concurrent::Context;
use leptos_reactive::RwSignal;
use nalgebra::{Matrix3, Point2, Rotation3, Scalar, Translation3, UnitVector3, Vector2, Vector3};
use opencv_ros_camera::RosOpenCvIntrinsics;
use parking_lot::Mutex;
use sqpnp::types::{SQPSolution, SolverParameters};
use std::sync::Arc;
use std::time::{Duration, UNIX_EPOCH};
use tokio_stream::StreamExt;
use tracing::{debug, info};

pub fn transform_aimpoint_to_identity(
    center_aim: Point2<f64>,
    p1: Point2<f64>,
    p2: Point2<f64>,
    p3: Point2<f64>,
    p4: Point2<f64>,
) -> Option<Point2<f64>> {
    ats_cv::transform_aim_point(
        center_aim,
        p1,
        p2,
        p3,
        p4,
        Point2::new(0.5, 1.),
        Point2::new(0., 0.5),
        Point2::new(0.5, 0.),
        Point2::new(1., 0.5),
    )
}

pub fn my_pnp(projections: &[Vector2<f64>]) -> Option<SQPSolution> {
    let _3dpoints = [
        Vector3::new(0.35 * 16. / 9., 0., 0.),
        Vector3::new(0.65 * 16. / 9., 0., 0.),
        Vector3::new(0.65 * 16. / 9., 1., 0.),
        Vector3::new(0.35 * 16. / 9., 1., 0.),
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
    if a[0].y < a[1].y {
        a.swap(0, 1);
    }
    if a[2].y > a[3].y {
        a.swap(2, 3);
    }
    if a[0].y < a[3].y {
        a.swap(0, 3);
    }
    if a[2].y > a[1].y {
        a.swap(2, 1);
    }
    if a[1].x > a[3].x {
        a.swap(1, 3);
    }
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
    if a[0].y > a[2].y {
        a.swap(0, 2);
    }
    if a[1].y > a[3].y {
        a.swap(1, 3);
    }
    if a[0].y > a[1].y {
        a.swap(0, 1);
    }
    if a[2].y > a[3].y {
        a.swap(2, 3);
    }
    if a[1].y > a[2].y {
        a.swap(1, 2);
    }
    if a[0].x > a[1].x {
        a.swap(0, 1);
    }
    if a[2].x < a[3].x {
        a.swap(2, 3);
    }
}

pub fn sort_points<T: Scalar + PartialOrd>(a: &mut [Point2<T>]) {
    sort_rectangle(a);
}

pub struct MotRunner {
    pub state: MotState,
    pub device: Option<UsbDevice>,
    pub general_config: GeneralConfig,
    pub record_impact: bool,
    pub record_packets: bool,
    pub datapoints: Arc<Mutex<Vec<crate::TestFrame>>>,
    pub packets: Arc<Mutex<Vec<(u128, ats_usb::packets::vm::PacketData)>>>,
    pub ui_update: RwSignal<()>,
    pub ui_ctx: Context,
    pub wfnf_realign: bool,
    pub screen_calibrations:
        ArrayVec<(u8, ScreenCalibration<f32>), { (ats_cv::foveated::MAX_SCREEN_ID + 1) as usize }>,
}

pub async fn run(runner: Arc<Mutex<MotRunner>>) {
    tokio::join!(
        combined_markers_loop(runner.clone()),
        accel_stream(runner.clone()),
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
        if let Some(mot_data) = mot_data_stream.next().await {
            let nf_data = mot_data.mot_data_nf;
            let wf_data = mot_data.mot_data_wf;
            let mut runner = runner.lock();
            let nf_data = ArrayVec::<MotData, 16>::from_iter(nf_data.into_iter());
            // let nf_data = ArrayVec::<MotData,16>::from_iter(dummy_nf_data());
            let wf_data = ArrayVec::<MotData, 16>::from_iter(wf_data.into_iter());

            let state = &mut runner.state;
            state.nf_data = Some(nf_data);
            state.wf_data = Some(wf_data);

            if runner.record_packets {
                runner.packets.lock().push((
                    std::time::SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_millis(),
                    ats_usb::packets::vm::PacketData::ObjectReport(mot_data),
                ));
            }
        }
    }
}

// fn get_raycast_aimpoint(fv_state: &ats_cv::foveated::FoveatedAimpointState, screen_calibration: &ScreenCalibration<f64>) -> (Rotation3<f64>, Translation3<f64>, Option<Point2<f64>>) {
//     let orientation = fv_state.filter.orientation.cast();
//     let position = fv_state.filter.position.cast();
//
//     let flip_yz = Matrix3::new(
//         1., 0., 0.,
//         0., -1., 0.,
//         0., 0., -1.,
//     );
//
//     let rot = Rotation3::from_matrix_unchecked(flip_yz * orientation.to_rotation_matrix() * flip_yz);
//     let trans = Translation3::from(flip_yz * position);
//
//     let isometry = nalgebra::Isometry::<f64, Rotation3<f64>, 3>::from_parts(trans, rot);
//
//     let fv_aimpoint = ats_cv::calculate_aimpoint(
//         &isometry,
//         screen_calibration,
//     );
//
//     (rot, trans, fv_aimpoint)
// }

fn my_raycast_update(runner: &mut MotRunner) {
    let screen_calibrations = runner.screen_calibrations.clone();
    let fv_state = &mut runner.state.fv_state;
    let offset = runner.state.fv_zero_offset;
    let (pose, aimpoint_and_d) = ats_cv::helpers::raycast_update(&screen_calibrations, fv_state, Some(offset));
    if let Some(pose) = pose {
        let flip_yz = Matrix3::new(1., 0., 0., 0., -1., 0., 0., 0., -1.);

        let rot = Rotation3::from_matrix_unchecked(flip_yz * pose.0 * flip_yz);
        let trans = Translation3::from(flip_yz * pose.1);
        runner.state.rotation_mat = *rot.matrix();
        runner.state.translation_mat = trans.vector;
    }
    if let Some(aimpoint_and_d) = aimpoint_and_d {
        runner.state.fv_aimpoint = aimpoint_and_d.0;
        runner.state.distance = aimpoint_and_d.1;
    }
}

async fn combined_markers_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().device.c().unwrap();
    let mut combined_markers_stream = device.stream_combined_markers().await.unwrap();

    while let Some(report) = combined_markers_stream.next().await {
        let CombinedMarkersReport {
            nf_points,
            wf_points,
        } = report;

        let mut runner = runner.lock();

        // Helper closure to process points
        let process_points = |points, camera_model, stereo_iso| {
            let point_tuples = filter_and_create_point_tuples(points);
            let points_raw: Vec<_> = point_tuples.iter().map(|&(_, p)| p).collect();
            let points_transformed = transform_points(&points_raw, camera_model);
            let intrinsics = ats_common::ros_opencv_intrinsics_type_convert(camera_model);
            let normalized_points: ArrayVec<_, 16> = points_transformed
                .iter()
                .map(|&p| to_normalized_image_coordinates(p, &intrinsics, stereo_iso))
                .collect();
            let markers = point_tuples
                .iter()
                .zip(&normalized_points)
                .map(|(&(mot_id, _), &normalized)| Marker {
                    mot_id,
                    pattern_id: None,
                    normalized,
                })
                .collect();
            (point_tuples, points_transformed, normalized_points, markers)
        };

        // Process nf_points and wf_points
        let (nf_point_tuples, nf_points_transformed, nf_normalized, nf_markers2) =
            process_points(&nf_points, &runner.general_config.camera_model_nf, None);
        let (wf_point_tuples, wf_points_transformed, wf_normalized, wf_markers2) = process_points(
            &wf_points,
            &runner.general_config.camera_model_wf,
            Some(&runner.general_config.stereo_iso.cast()),
        );

        runner.state.nf_markers2 = nf_markers2;
        runner.state.wf_markers2 = wf_markers2;

        let gravity_vec = UnitVector3::new_unchecked(
            runner
                .state
                .orientation
                .inverse_transform_vector(&Vector3::z_axis())
                .xzy(),
        );

        // Re-alignment logic
        if runner.wfnf_realign {
            if let Some((wf_match_ix, _, _)) = ats_cv::foveated::identify_markers(
                &wf_normalized,
                gravity_vec.cast(),
                &runner.screen_calibrations,
            ) {
                let wf_match = wf_match_ix.map(|i| wf_normalized[i].coords);
                let (nf_match_ix, _) = ats_cv::foveated::match3(&nf_normalized, &wf_match);
                if nf_match_ix.iter().all(Option::is_some) {
                    let nf_ordered =
                        nf_match_ix.map(|i| nf_normalized[i.unwrap()].coords.push(1.0));
                    let wf_ordered = wf_match_ix.map(|i| wf_normalized[i].coords.push(1.0));
                    let q = calculate_rotational_offset(&wf_ordered, &nf_ordered);
                    runner.general_config.stereo_iso.rotation *= q.cast();
                    runner.wfnf_realign = false;
                }
            }
        }

        // Observe markers
        let nf_markers_cv = runner
            .state
            .nf_markers2
            .iter()
            .map(|m| m.ats_cv_marker())
            .collect::<ArrayVec<_, 16>>();
        let wf_markers_cv = runner
            .state
            .wf_markers2
            .iter()
            .map(|m| m.ats_cv_marker())
            .collect::<ArrayVec<_, 16>>();
        let screen_calibrations = runner.screen_calibrations.clone();
        runner.state.fv_state.observe_markers(
            &nf_markers_cv,
            &wf_markers_cv,
            gravity_vec.cast(),
            &screen_calibrations,
        );

        my_raycast_update(&mut runner);

        let wf_markers: Option<(
            [usize; MARKER_PATTERN_LEN],
            [Vector2<f32>; MARKER_PATTERN_LEN],
            Option<u8>,
        )> = None;

        let (wf_marker_ix, wf_reproj) = wf_markers
            .map(|(markers, reproj, _)| (markers.to_vec(), reproj.to_vec()))
            .unwrap_or_default();

        // Match nf_markers with wf_markers
        let mut nf_markers = ArrayVec::<Point2<f32>, 16>::new();
        if wf_marker_ix.len() >= MARKER_PATTERN_LEN {
            let chosen_wf_markers: [_; MARKER_PATTERN_LEN] = wf_marker_ix
                .iter()
                .map(|&i| wf_normalized[i].coords)
                .collect::<ArrayVec<_, MARKER_PATTERN_LEN>>()
                .into_inner()
                .unwrap();

            let (nf_match_ix, _) = ats_cv::foveated::match3(&nf_normalized, &chosen_wf_markers);

            for (i, &wf_idx) in wf_marker_ix.iter().enumerate() {
                runner.state.wf_markers2[wf_idx].pattern_id = Some(i as u8);
                if let Some(nf_idx) = nf_match_ix[i] {
                    nf_markers.push(nf_points_transformed[nf_idx]);
                    runner.state.nf_markers2[nf_idx].pattern_id = Some(i as u8);
                } else {
                    nf_markers.push(Point2::new(-9999., -9999.));
                }
            }
        }

        // Update runner state
        runner.state.nf_points = nf_point_tuples
            .into_iter()
            .filter(|&(mot_id, p)| {
                !runner
                    .state
                    .nf_markers2
                    .iter()
                    .any(|m| m.mot_id == mot_id && nf_markers.contains(&p))
            })
            .collect();

        runner.state.wf_points = wf_point_tuples
            .into_iter()
            .enumerate()
            .filter(|(i, _)| !wf_marker_ix.contains(i))
            .map(|(_, p)| p)
            .collect();

        runner.state.nf_markers = nf_markers;
        runner.state.wf_markers = wf_marker_ix
            .iter()
            .map(|&i| wf_points_transformed[i])
            .collect();
        runner.state.wf_reproj = wf_reproj.into_iter().map(Into::into).collect();

        // Update aimpoint history
        let gravity_angle = -gravity_vec.z.atan2(-gravity_vec.x).to_degrees() + 90.0;
        let index = runner.state.fv_aimpoint_history_index;
        runner.state.fv_aimpoint_history[index] = (runner.state.fv_aimpoint, gravity_angle);
        runner.state.fv_aimpoint_history_index =
            (index + 1) % runner.state.fv_aimpoint_history.len();

        // Record packets if enabled
        if runner.record_packets {
            runner.packets.lock().push((
                std::time::SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_millis(),
                ats_usb::packets::vm::PacketData::CombinedMarkersReport(report),
            ));
        }
    }
}

fn filter_and_create_point_tuples(points: &[Point2<u16>]) -> Vec<(u8, Point2<f32>)> {
    points
        .iter()
        .enumerate()
        .filter_map(|(id, pos)| {
            if (100..3996).contains(&pos.x) && (100..3996).contains(&pos.y) {
                Some((id as u8, Point2::new(pos.x as f32, pos.y as f32)))
            } else {
                None
            }
        })
        .collect()
}

fn transform_points(
    points: &[Point2<f32>],
    camera_intrinsics: &RosOpenCvIntrinsics<f32>,
) -> Vec<Point2<f32>> {
    ats_cv::undistort_points(
        &ats_common::ros_opencv_intrinsics_type_convert(camera_intrinsics),
        points,
    )
}

async fn accel_stream(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().device.c().unwrap();
    let mut accel_stream = device.stream_accel().await.unwrap();
    let mut prev_timestamp = None;
    while let Some(accel) = accel_stream.next().await {
        let mut runner = runner.lock();
        let accel_odr = runner.general_config.accel_config.accel_odr;

        // correct accel and gyro bias and scale
        let accel = ats_usb::packets::vm::AccelReport {
            accel: accel.corrected_accel(&runner.general_config.accel_config),
            gyro: accel.corrected_gyro(&runner.general_config.gyro_config),
            timestamp: accel.timestamp,
        };

        // println!("{:7.3?} {:7.3?}", accel.accel.xzy(), accel.gyro.xzy());
        // println!("{:7.3?}", accel.accel.norm());

        // print rotation in degrees
        // println!("Rotation: {}", accel.gyro.xzy().map(|x| x.to_degrees()));

        if let Some(_prev_timestamp) = prev_timestamp {
            if accel.timestamp < _prev_timestamp {
                prev_timestamp = None;
                continue;
            }
        }

        if let Some(prev_timestamp) = prev_timestamp {
            let elapsed = accel.timestamp as u64 - prev_timestamp as u64;
            // println!("elapsed: {}", elapsed);
            runner.state.fv_state.predict(
                -accel.accel.xzy(),
                -accel.gyro.xzy(),
                Duration::from_micros(elapsed),
            );

            let sample_period = runner.state.madgwick.sample_period_mut();
            *sample_period = elapsed as f32 / 1_000_000.;
        } else {
            runner.state.fv_state.predict(
                -accel.accel.xzy(),
                -accel.gyro.xzy(),
                Duration::from_secs_f32(1. / accel_odr as f32),
            );
        }
        prev_timestamp = Some(accel.timestamp);

        let _ = runner
            .state
            .madgwick
            .update_imu(&Vector3::from(accel.gyro), &Vector3::from(accel.accel));
        runner.state.orientation = runner.state.madgwick.quat.to_rotation_matrix();

        ats_cv::series_add!(
            imu_data,
            (-accel.accel.xzy().cast(), -accel.gyro.xzy().cast())
        );

        my_raycast_update(&mut runner);

        if runner.record_packets {
            runner.packets.lock().push((
                std::time::SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_millis(),
                ats_usb::packets::vm::PacketData::AccelReport(accel),
            ));
        }
    }
}

// todo use an aimpoint history to choose the aimpoint closest to the timestamp
async fn impact_loop(runner: Arc<Mutex<MotRunner>>) {
    let device = runner.lock().device.c().unwrap();
    let mut impact_stream = device.stream_impact().await.unwrap();
    while let Some(_impact) = impact_stream.next().await {
        let runner = runner.lock();
        if runner.record_impact {
            let mut frame = TestFrame {
                fv_aimpoint_x: None,
                fv_aimpoint_y: None,
                opposite_cant: None,
            };

            {
                let data = runner.state.fv_aimpoint_history[runner.state.fv_aimpoint_history_index];
                let fv_aimpoint = data.0;
                frame.fv_aimpoint_x = Some(fv_aimpoint.x);
                frame.fv_aimpoint_y = Some(fv_aimpoint.y);
                frame.opposite_cant = Some(data.1);
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
        if runner.record_packets {
            runner.packets.lock().push((
                std::time::SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_millis(),
                ats_usb::packets::vm::PacketData::ImpactReport(_impact),
            ));
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
