use std::sync::Arc;
use arrayvec::ArrayVec;
use ats_cv::choose_rectangle_nearfield_markers;
use nalgebra::{Point2, Scale2};
use tokio::sync::Mutex;
use iui::controls::{Area, AreaDrawParams, AreaHandler};
use iui::draw::{Brush, FillMode, Path, SolidBrush, StrokeParams};
use iui::UI;
use crate::custom_shapes::{draw_crosshair, draw_diamond, draw_grid};
use crate::mot_runner::{rescale, sort_points, MotRunner};
use crate::packet::MarkerPattern;

pub struct RunCanvas {
    pub ctx: UI,
    pub state: Arc<Mutex<MotRunner>>,
}

impl AreaHandler for RunCanvas {
    fn draw(&mut self, _area: &Area, draw_params: &AreaDrawParams) {
        let ctx = &draw_params.context;

        let ch_path = Path::new(ctx, FillMode::Winding);
        let nf_path = Path::new(ctx, FillMode::Winding);
        let wf_path = Path::new(ctx, FillMode::Winding);
        let nf_grid_path = Path::new(ctx, FillMode::Winding);
        let runner = self.state.blocking_lock();
        let state = &runner.state;
        if let Some(nf_data) = state.nf_data.as_ref() {
            let mut nf_points = ArrayVec::<Point2<f64>,16>::new();
            for (i, mot_data) in nf_data.iter().enumerate() {
                if mot_data.area == 0 {
                    break;
                }
                nf_points.push(Point2::new(mot_data.cx, mot_data.cy).cast());
                let x = mot_data.cx as f64 / 4095. * draw_params.area_width;
                let y = mot_data.cy as f64 / 4095. * draw_params.area_height;

                let left = mot_data.boundary_left as f64 / 98. * draw_params.area_width;
                let down = mot_data.boundary_down as f64 / 98. * draw_params.area_height;
                let right = mot_data.boundary_right as f64 / 98. * draw_params.area_width;
                let up = mot_data.boundary_up as f64 / 98. * draw_params.area_height;

                draw_crosshair(&ctx, &ch_path, x, y, 50.);

                nf_path.add_rectangle(
                    ctx,
                    left,
                    up,
                    right-left,
                    down-up,
                );

                ctx.draw_text(x+20.0, y+20.0, format!("({}, {}) id={i}", mot_data.cx, mot_data.cy).as_str());
            }
            if nf_points.len() >= 4 {
                let mut chosen = choose_rectangle_nearfield_markers(&mut nf_points, state.screen_id);
                let points = match chosen.as_mut() {
                    Some(p) if runner.general_config.marker_pattern == MarkerPattern::Rectangle => p,
                    _ => &mut nf_points[..4],
                };
                // todo don't use hardcoded 4095x4095 res assumption
                for p in points.iter_mut() {
                    *p = Scale2::new(draw_params.area_width, draw_params.area_height) * *p / 4095.;
                }

                sort_points(points, runner.general_config.marker_pattern);
                let top = runner.markers_settings.views[0].marker_top.position;
                let left = runner.markers_settings.views[0].marker_left.position;
                let bottom = runner.markers_settings.views[0].marker_bottom.position;
                let right = runner.markers_settings.views[0].marker_right.position;
                let transform = ats_cv::get_perspective_transform(
                    points[0], points[1],
                    points[2], points[3],
                    Point2::new(rescale(bottom.x as f64), rescale(bottom.y as f64)), // bottom
                    Point2::new(rescale(left.x as f64), rescale(left.y as f64)), // left
                    Point2::new(rescale(top.x as f64), rescale(top.y as f64)), // top
                    Point2::new(rescale(right.x as f64), rescale(right.y as f64)), // right
                );
                if let Some(transform) = transform.and_then(|t| t.try_inverse()) {
                    draw_grid(ctx, &nf_grid_path, 10, 10, transform);
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
                // todo don't use hardcoded 4095x4095 res assumption
                let x = mot_data.cx as f64 / 4095. * draw_params.area_width;
                let y = mot_data.cy as f64 / 4095. * draw_params.area_height;

                let left = mot_data.boundary_left as f64 / 98. * draw_params.area_width;
                let down = mot_data.boundary_down as f64 / 98. * draw_params.area_height;
                let right = mot_data.boundary_right as f64 / 98. * draw_params.area_width;
                let up = mot_data.boundary_up as f64 / 98. * draw_params.area_height;

                draw_crosshair(&ctx, &ch_path, x, y, 50.);

                wf_path.add_rectangle(
                    ctx,
                    left,
                    up,
                    right-left,
                    down-up,
                );
            }
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
        let stroke = StrokeParams {
            cap: 0, // Bevel
            join: 0, // Flat
            thickness: 2.,
            miter_limit: 0.,
            dashes: vec![],
            dash_phase: 0.,
        };
        let center_point_path = Path::new(ctx, FillMode::Winding);
        draw_diamond(ctx, &center_point_path, 0.5 * draw_params.area_width, 0.5 * draw_params.area_height, 8.0, 8.0);
        center_point_path.end(ctx);
        ctx.stroke(&center_point_path, &brush, &stroke);
    }
}
