use std::f64::consts::PI;
use std::sync::Arc;
use arrayvec::ArrayVec;
use ats_cv::choose_rectangle_nearfield_markers;
use nalgebra::{Point2, Rotation2, Scale2, Transform2, Translation2, Vector2, Vector3};
use parking_lot::Mutex;
use iui::controls::{Area, AreaDrawParams, AreaHandler};
use iui::draw::{Brush, FillMode, Path, SolidBrush, StrokeParams};
use iui::UI;
use crate::custom_shapes::{draw_crosshair, draw_crosshair_rotated, draw_diamond, draw_grid, draw_square};
use crate::mot_runner::{rescale, sort_points, MotRunner};
use crate::packet::MarkerPattern;

pub struct RunCanvas {
    pub ctx: UI,
    pub runner: Arc<Mutex<MotRunner>>,
}

impl AreaHandler for RunCanvas {
    fn draw(&mut self, _area: &Area, draw_params: &AreaDrawParams) {
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
        let runner = self.runner.lock();
        let state = &runner.state;

        let gravity: Vector3<f64> = state.orientation.quat.inverse_transform_vector(&Vector3::z()).into();
        let gravity_angle = -gravity.x.atan2(gravity.z);

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
        let gravity_rot = Rotation2::new(-gravity_angle);
        for (i, point) in state.nf_points.iter().enumerate() {
            // todo don't use hardcoded 4095x4095 res assumption
            let p = point / 4095.
                - Vector2::new(0.5, 0.5);
            let p = gravity_rot * p;
            let p = draw_tf * p;

            draw_crosshair(&ctx, &ch_path, p.x, p.y, 50.);

            ctx.draw_text(p.x+20.0, p.y+20.0, format!("({}, {}) id={i}", point.x, point.y).as_str());
        }
        ctx.draw_text(
            20.0,
            20.0,
            &format!("screen_id = {}", runner.state.screen_id),
        );
        if state.nf_points.len() >= 4 {
            let mut points = state.nf_points.clone();
            let mut chosen = choose_rectangle_nearfield_markers(&mut points, state.screen_id);
            let points = match chosen.as_mut() {
                // Some(p) if runner.general_config.marker_pattern == MarkerPattern::Rectangle => p,
                _ => &mut points[..4],
            };
            // sort_points(points, runner.general_config.marker_pattern);

            let top = runner.markers_settings.views[0].marker_top.position;
            let left = runner.markers_settings.views[0].marker_left.position;
            let bottom = runner.markers_settings.views[0].marker_bottom.position;
            let right = runner.markers_settings.views[0].marker_right.position;
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
        nf_path.end(ctx);
        nf_grid_path.end(ctx);
        for point in &state.wf_points {
            let magic = 4.5;
            let p = Point2::new(point.x, point.y);
            // scale p by magic where 2048,2048 is the center
            let p = (p - Point2::new(2048., 2048.)) * magic;
            // todo don't use hardcoded 4095x4095 res assumption
            let p = Point2::new(p.x + 2048., p.y + 2048.) / 4095.
                - Vector2::new(0.5, 0.5);
            let p = gravity_rot * p;
            let p = draw_tf * p;

            draw_crosshair_rotated(&ctx, &ch_path, p.x, p.y, 50.);
        }
        wf_path.end(ctx);

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
        ctx.stroke(&nf_grid_path, &brush, &stroke);

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
}
