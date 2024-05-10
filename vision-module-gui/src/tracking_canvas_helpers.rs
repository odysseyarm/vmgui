use std::f64::consts::PI;
use std::sync::Arc;
use arrayvec::ArrayVec;
use nalgebra::{Point2, Rotation2, Scale2, Transform2, Translation2, Vector2, Vector3};
use parking_lot::Mutex;
use iui::controls::{Area, AreaDrawParams};
use iui::draw::{Brush, DrawContext, FillMode, Path, SolidBrush, StrokeParams};
use iui::UI;
use crate::custom_shapes::{self, draw_crosshair_rotated, draw_diamond, draw_grid, draw_square};
use crate::marker_config_window::MarkersSettings;
use crate::mot_runner::{rescale, MotRunner};
use crate::MotState;

pub fn draw(ctx: UI, runner: Arc<Mutex<MotRunner>>, _area: &Area, draw_params: &AreaDrawParams, raw: bool) {
    let ctx = &draw_params.context;
    let awidth = draw_params.area_width;
    let aheight = draw_params.area_height;
    let draw_size = (awidth.min(aheight).powi(2)/2.0).sqrt();
    let stroke2 = StrokeParams {
        cap: 0, // Bevel
        join: 0, // Flat
        thickness: 2.,
        miter_limit: 0.,
        dashes: vec![],
        dash_phase: 0.,
    };
    let stroke1 = StrokeParams {
        thickness: 1.,
        ..stroke2.clone()
    };

    let border_path = Path::new(ctx, FillMode::Winding);
    let ch_path = Path::new(ctx, FillMode::Winding);
    let nf_path = Path::new(ctx, FillMode::Winding);
    let wf_path = Path::new(ctx, FillMode::Winding);
    let nf_grid_path = Path::new(ctx, FillMode::Winding);
    let runner = runner.lock();
    let state = &runner.state;

    let gravity_vec = state.orientation.inverse_transform_vector(&Vector3::z());
	let gravity_angle = f64::atan2(-gravity_vec.z as f64, -gravity_vec.x as f64) + PI/2.;

    // Border around the square drawing area
    {
        draw_square(ctx, &border_path, Transform2::from_matrix_unchecked(
            Translation2::new(awidth/2., aheight/2.).to_homogeneous()
            * Rotation2::new(-gravity_angle).to_homogeneous()
            * Scale2::new(draw_size, draw_size).to_homogeneous()
        ));
        border_path.end(ctx);
        ctx.stroke(&border_path, &Brush::Solid(SolidBrush { r: 0., g: 0., b: 0., a: 1. }), &stroke1);
    }

    // Green line representing the up direction relative to the vision module.
    {
        let gravity_line_path = Path::new(ctx, FillMode::Winding);
        gravity_line_path.new_figure(ctx, 0.5 * draw_params.area_width, 0.5 * draw_params.area_height);
        let angle = -gravity_angle - PI/2.;
        gravity_line_path.line_to(
            ctx,
            0.5 * draw_params.area_width + 50.0 * angle.cos(),
            0.5 * draw_params.area_height + 50.0 * angle.sin(),
        );
        gravity_line_path.end(ctx);
        ctx.stroke(&gravity_line_path, &Brush::Solid(SolidBrush { r: 0., g: 1., b: 0., a: 1. }), &stroke2);
    }

    let draw_tf = Transform2::from_matrix_unchecked(
        Translation2::new(awidth/2.0, aheight/2.0).to_homogeneous()
        * Scale2::new(draw_size, draw_size).to_homogeneous()
    );

    ctx.draw_text(
        20.0,
        20.0,
        &format!("screen_id = {}", state.screen_id),
    );

    let gravity_rot = Rotation2::new(-gravity_angle);
    if raw {
        draw_raw(ctx, state, draw_tf, gravity_rot, &nf_path, &wf_path, &nf_grid_path, &runner.markers_settings, &ch_path);
    } else {
        draw_not_raw(ctx, state, &runner.general_config, draw_tf, gravity_rot, &nf_path, &wf_path, &nf_grid_path, &runner.markers_settings, &ch_path);
    }

    ch_path.end(ctx);

    let brush = Brush::Solid(SolidBrush {
        r: 1.,
        g: 0.,
        b: 0.,
        a: 0.5,
    });

    ctx.fill(&nf_path, &brush);

    let brush = Brush::Solid(SolidBrush {
        r: 0.,
        g: 0.,
        b: 1.,
        a: 0.5,
    });

    ctx.fill(&wf_path, &brush);

    let brush = Brush::Solid(SolidBrush {
        r: 0.,
        g: 0.,
        b: 0.,
        a: 1.,
    });

    let stroke = StrokeParams {
        cap: 0, // Bevel
        join: 0, // Flat
        thickness: 6.,
        miter_limit: 0.,
        dashes: vec![],
        dash_phase: 0.,
    };

    ctx.stroke(&ch_path, &brush, &stroke);

    // Grid
    let brush = Brush::Solid(SolidBrush {
        r: 0.5,
        g: 0.,
        b: 0.,
        a: 1.,
    });
    let stroke = StrokeParams {
        cap: 0, // Bevel
        join: 0, // Flat
        thickness: 1.,
        miter_limit: 0.,
        dashes: vec![],
        dash_phase: 0.,
    };
    // ctx.stroke(&nf_grid_path, &brush, &stroke);

    // Center point
    let brush = Brush::Solid(SolidBrush {
        r: 0.0,
        g: 0.,
        b: 0.,
        a: 1.,
    });
    let center_point_path = Path::new(ctx, FillMode::Winding);
    draw_diamond(ctx, &center_point_path, 0.5 * draw_params.area_width, 0.5 * draw_params.area_height, 8.0, 8.0);
    center_point_path.end(ctx);
    ctx.stroke(&center_point_path, &brush, &stroke2);
}

fn draw_raw(ctx: &DrawContext, state: &MotState, draw_tf: Transform2<f64>, gravity_rot: Rotation2<f64>, nf_path: &Path, wf_path: &Path, nf_grid_path: &Path, markers_settings: &MarkersSettings, ch_path: &Path) {
    if let Some(nf_data) = state.nf_data.as_ref() {
        let mut nf_points = ArrayVec::<Point2<f64>,16>::new();
        for (i, mot_data) in nf_data.iter().enumerate() {
            if mot_data.area == 0 {
                break;
            }
            // todo don't use hardcoded 4095x4095 res assumption
            let p = Point2::new(mot_data.cx, mot_data.cy).cast::<f64>() / 4095.
                - Vector2::new(0.5, 0.5);
            let p = gravity_rot * p;
            nf_points.push((p + Vector2::new(0.5, 0.5)) * 4095.);
            let p = draw_tf * p;

            let left = mot_data.boundary_left as f64 / 98.;
            let down = mot_data.boundary_down as f64 / 98.;
            let right = mot_data.boundary_right as f64 / 98.;
            let up = mot_data.boundary_up as f64 / 98.;

            custom_shapes::draw_rectangle(ctx, &nf_path, &[left, down, right, up], &gravity_rot, &draw_tf);
            custom_shapes::draw_marker(ctx, &ch_path, p, &format!("({}, {}) id={}", mot_data.cx, mot_data.cy, i));
        }

        if nf_points.len() >= 4 {
            let mut chosen = ats_cv::choose_rectangle_markers(&mut nf_points, state.screen_id, 300.);
            let points = match chosen.as_mut() {
                // Some(p) if runner.general_config.marker_pattern == MarkerPattern::Rectangle => p,
                _ => &mut nf_points[..4],
            };
            // sort_points(points, runner.general_config.marker_pattern);

            let top = markers_settings.views[0].marker_top.position;
            let left = markers_settings.views[0].marker_left.position;
            let bottom = markers_settings.views[0].marker_bottom.position;
            let right = markers_settings.views[0].marker_right.position;
            let transform = ats_cv::get_perspective_transform(
                Point2::new(rescale(bottom.x as f64), rescale(bottom.y as f64)), // bottom
                Point2::new(rescale(left.x as f64), rescale(left.y as f64)), // left
                Point2::new(rescale(top.x as f64), rescale(top.y as f64)), // top
                Point2::new(rescale(right.x as f64), rescale(right.y as f64)), // right
                points[0], points[1],
                points[2], points[3],
            );
            if let Some(transform) = transform {
                draw_grid(ctx, &nf_grid_path, 10, 10, draw_tf.to_homogeneous() * Scale2::new(1./4095., 1./4095.).to_homogeneous() * transform);
            }
        }
    }
    nf_path.end(ctx);
    nf_grid_path.end(ctx);

    if let Some(wf_data) = state.wf_data.as_ref() {
        for mot_data in wf_data {
            if mot_data.area == 0 {
                break;
            }

            let p = Point2::new(mot_data.cx, mot_data.cy).cast::<f64>() / 4095.
                - Vector2::new(0.5, 0.5);
            let p = gravity_rot * p;
            let p = draw_tf * p;

            let left = mot_data.boundary_left as f64 / 98.;
            let down = mot_data.boundary_down as f64 / 98.;
            let right = mot_data.boundary_right as f64 / 98.;
            let up = mot_data.boundary_up as f64 / 98.;

            custom_shapes::draw_rectangle(ctx, &wf_path, &[left, down, right, up], &gravity_rot, &draw_tf);

            draw_crosshair_rotated(&ctx, &ch_path, p.x, p.y, 50.);
        }
    }
    wf_path.end(ctx);
}

fn draw_not_raw(ctx: &DrawContext, state: &MotState, config: &ats_usb::packet::GeneralConfig, draw_tf: Transform2<f64>, gravity_rot: Rotation2<f64>, nf_path: &Path, wf_path: &Path, nf_grid_path: &Path, markers_settings: &MarkersSettings, ch_path: &Path) {
    for (i, point) in state.nf_points.iter().enumerate() {
        // todo don't use hardcoded 4095x4095 res assumption
        let p = point / 4095.
            - Vector2::new(0.5, 0.5);
        let p = gravity_rot * p;
        let p = draw_tf * p;

        custom_shapes::draw_marker(ctx, &ch_path, p, &format!("({}, {}) id={}", point.x, point.y, i));
    }
    nf_path.end(ctx);

    let wf_to_nf_points = ats_cv::wf_to_nf_points(&state.wf_points, &ats_cv::ros_opencv_intrinsics_type_convert(&config.camera_model_nf), &ats_cv::ros_opencv_intrinsics_type_convert(&config.camera_model_wf), config.stereo_iso.cast());
    for point in wf_to_nf_points {
        // todo don't use hardcoded 4095x4095 res assumption
        let p = point / 4095.
            - Vector2::new(0.5, 0.5);
        let p = gravity_rot * p;
        let p = draw_tf * p;

        draw_crosshair_rotated(&ctx, &ch_path, p.x, p.y, 50.);
    }
    wf_path.end(ctx);

    if state.nf_points.len() >= 4 {
        let mut points = state.nf_points.clone();
        let mut chosen = ats_cv::choose_rectangle_markers(&mut points, state.screen_id, 300.);
        let points = match chosen.as_mut() {
            // Some(p) if runner.general_config.marker_pattern == MarkerPattern::Rectangle => p,
            _ => &mut points[..4],
        };
        // sort_points(points, runner.general_config.marker_pattern);

        let top = markers_settings.views[0].marker_top.position;
        let left = markers_settings.views[0].marker_left.position;
        let bottom = markers_settings.views[0].marker_bottom.position;
        let right = markers_settings.views[0].marker_right.position;
        let transform = ats_cv::get_perspective_transform(
            Point2::new(rescale(bottom.x as f64), rescale(bottom.y as f64)), // bottom
            Point2::new(rescale(left.x as f64), rescale(left.y as f64)), // left
            Point2::new(rescale(top.x as f64), rescale(top.y as f64)), // top
            Point2::new(rescale(right.x as f64), rescale(right.y as f64)), // right
            points[0], points[1],
            points[2], points[3],
        );
        if let Some(transform) = transform {
            draw_grid(ctx, &nf_grid_path, 10, 10, draw_tf.to_homogeneous() * Scale2::new(1./4095., 1./4095.).to_homogeneous() * transform);
        }
    }
    nf_grid_path.end(ctx);
}
