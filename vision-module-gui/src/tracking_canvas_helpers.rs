use std::f64::consts::PI;
use std::sync::Arc;
use arrayvec::ArrayVec;
use nalgebra::{Isometry3, Point2, Point3, Rotation2, Scale2, Transform2, Translation2, Vector2, Vector3};
use parking_lot::Mutex;
use iui::controls::{Area, AreaDrawParams};
use iui::draw::{Brush, DrawContext, FillMode, Path, SolidBrush, StrokeParams};
use iui::UI;
use crate::custom_shapes::{self, draw_crosshair_rotated, draw_diamond, draw_square, draw_text, solid_brush};
use crate::mot_runner::{MotRunner};
use crate::MotState;

pub fn draw(ctx: UI, runner: Arc<Mutex<MotRunner>>, _area: &Area, draw_params: &AreaDrawParams, raw: bool) {
    let ctx = &draw_params.context;
    let awidth = draw_params.area_width;
    let aheight = draw_params.area_height;
    let draw_size = (awidth.min(aheight).powi(2)/2.0).sqrt();
    let draw_size = if raw {
        draw_size
    } else {
        // draw_size / 2.0
        draw_size
    };
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

    draw_text(
        &ctx,
        20.0,
        20.0,
        &format!("screen_id = {}", state.screen_id),
    );

    let gravity_rot = Rotation2::new(-gravity_angle);
    if raw {
        draw_raw(ctx, state, draw_tf, gravity_rot, &nf_path, &wf_path, &nf_grid_path, &ch_path);
    } else {
        draw_not_raw(ctx, state, &runner.general_config, draw_tf, gravity_rot, &nf_path, &wf_path, &nf_grid_path, &ch_path, &runner.screen_info.marker_points);
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
        thickness: 2.,
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

fn draw_raw(ctx: &DrawContext, state: &MotState, draw_tf: Transform2<f64>, gravity_rot: Rotation2<f64>, nf_path: &Path, wf_path: &Path, nf_grid_path: &Path, ch_path: &Path) {
    if let Some(nf_data) = state.nf_data.as_ref() {
        let mut nf_points = ArrayVec::<Point2<f64>,16>::new();
        for (i, mot_data) in nf_data.iter().enumerate() {
            if mot_data.area == 0 {
                continue;
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
            custom_shapes::draw_marker(ctx, &ch_path, p, &format!("({:.3}, {:.3}) id={}", mot_data.cx, mot_data.cy, i));
        }
    }
    nf_path.end(ctx);
    nf_grid_path.end(ctx);

    if let Some(wf_data) = state.wf_data.as_ref() {
        for mot_data in wf_data {
            if mot_data.area == 0 {
                continue;
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

fn draw_not_raw(ctx: &DrawContext, state: &MotState, config: &ats_usb::packet::GeneralConfig, draw_tf: Transform2<f64>, gravity_rot: Rotation2<f64>, nf_path: &Path, wf_path: &Path, nf_grid_path: &Path, ch_path: &Path, marker_points: &[Point3<f64>]) {
    nf_path.end(ctx);
    wf_path.end(ctx);
    let fx = config.camera_model_nf.p.m11 as f64;
    let fy = config.camera_model_nf.p.m22 as f64;
    let normalized_scale = Scale2::new(fx / 98.0, fy / 98.0);

    // let nf_points = state.nf_points.clone().iter().map(|x| x.2).collect::<Vec<_>>();
    let thin = StrokeParams {
        cap: 0, // Bevel
        join: 0, // Flat
        thickness: 1.,
        miter_limit: 0.,
        dashes: vec![],
        dash_phase: 0.,
    };
    let thick2 = StrokeParams {
        cap: 0, // Bevel
        join: 0, // Flat
        thickness: 2.,
        miter_limit: 0.,
        dashes: vec![],
        dash_phase: 0.,
    };
    let thick3 = StrokeParams {
        thickness: 3.,
        ..thick2.clone()
    };

    for marker in state.nf_markers2.iter() {
        let p = normalized_scale * marker.normalized;
        let p = gravity_rot * p;
        let p = draw_tf * p;

        let marker_path = Path::new(ctx, FillMode::Winding);
        custom_shapes::draw_marker(ctx, &marker_path, p, &format!("nf: sid={}", marker.screen_id));
        marker_path.end(ctx);
        match marker.pattern_id {
            Some(0 | 3) => ctx.stroke(&marker_path, &solid_brush(1.0, 0.0, 0.0), &thin),
            Some(1 | 4) => ctx.stroke(&marker_path, &solid_brush(0.0, 1.0, 0.0), &thin),
            Some(2 | 5) => ctx.stroke(&marker_path, &solid_brush(0.0, 0.0, 1.0), &thin),
            Some(_) => ctx.stroke(&marker_path, &solid_brush(1.0, 0.0, 1.0), &thin),
            None => ctx.stroke(&marker_path, &solid_brush(0.0, 0.0, 0.0), &thin),
        }
    }

    for marker in state.wf_markers2.iter() {
        let p = normalized_scale * marker.normalized;
        let p = gravity_rot * p;
        let p = draw_tf * p;

        let marker_path = Path::new(ctx, FillMode::Winding);
        custom_shapes::draw_marker_rotated(ctx, &marker_path, p, &format!("wf: sid={}", marker.screen_id));
        marker_path.end(ctx);
        match marker.pattern_id {
            Some(0 | 3) => ctx.stroke(&marker_path, &solid_brush(1.0, 0.0, 0.0), &thin),
            Some(1 | 4) => ctx.stroke(&marker_path, &solid_brush(0.0, 1.0, 0.0), &thin),
            Some(2 | 5) => ctx.stroke(&marker_path, &solid_brush(0.0, 0.0, 1.0), &thin),
            Some(_) => ctx.stroke(&marker_path, &solid_brush(1.0, 0.0, 1.0), &thin),
            None => ctx.stroke(&marker_path, &solid_brush(1.0, 0.0, 0.0), &thin),
        }
    }

    for p in marker_points { // eskf reprojections
        let position = state.fv_state.filter.position;
        let orientation = state.fv_state.filter.orientation;
        let reproj_tf = Isometry3::from_parts(position.into(), orientation);
        let fv_reproj_path = Path::new(ctx, FillMode::Winding);
        let p = reproj_tf.cast().inverse_transform_point(p);
        let p = p / p.z;
        // todo don't use hardcoded 4095x4095 res assumption
        let p = Point2::new(p.x*fx, p.y*fy) / 98.0 * 4095.0;
        let p = p / 4095.;
        let p = gravity_rot * p;
        let p = draw_tf * p;
        draw_crosshair_rotated(&ctx, &fv_reproj_path, p.x, p.y, 20.);
        fv_reproj_path.end(&ctx);
        ctx.stroke(&fv_reproj_path, &solid_brush(0.0, 0.69, 0.42), &thick3);
    }
    let pnp_iso = ats_cv::telemetry::pnp_solutions().get_last();
    if let Some(pnp_iso) = pnp_iso {
        for p in marker_points {
            let reproj_tf = pnp_iso.inverse();
            let pnp_reproj_path = Path::new(ctx, FillMode::Winding);
            let p = reproj_tf.cast().inverse_transform_point(p);
            let p = p / p.z;
            // todo don't use hardcoded 4095x4095 res assumption
            let p = Point2::new(p.x*fx, p.y*fy) / 98.0 * 4095.0;
            let p = p / 4095.;
            let p = gravity_rot * p;
            let p = draw_tf * p;
            draw_crosshair_rotated(&ctx, &pnp_reproj_path, p.x, p.y, 20.);
            pnp_reproj_path.end(&ctx);
            ctx.stroke(&pnp_reproj_path, &solid_brush(0.3, 0.3, 0.3), &thick3);
        }
    }
    for p in &state.wf_reproj {
        let wf_reproj_path = Path::new(ctx, FillMode::Winding);
        // todo don't use hardcoded 4095x4095 res assumption
        let p = Point2::new(p.x*fx, p.y*fy) / 98.0 * 4095.0;
        let p = p / 4095.;
        let p = gravity_rot * p;
        let p = draw_tf * p;
        draw_crosshair_rotated(&ctx, &wf_reproj_path, p.x, p.y, 20.);
        wf_reproj_path.end(&ctx);
        ctx.stroke(&wf_reproj_path, &solid_brush(0.627, 0.125, 0.941), &thick2);
    }
}
