use std::sync::Arc;
use tokio::sync::Mutex;
use iui::controls::{Area, AreaDrawParams, AreaHandler, AreaKeyEvent, Window};
use iui::draw::{Brush, FillMode, Path, SolidBrush, StrokeParams};
use iui::UI;
use crate::custom_shapes::draw_crosshair;
use crate::MotState;

pub struct TestCanvas {
    pub ctx: UI,
    pub window: Window,
    pub on_closing: Box<dyn FnMut(&mut Window)>,
    pub state: Arc<Mutex<MotState>>,
}

impl AreaHandler for TestCanvas {
    fn draw(&mut self, _area: &Area, draw_params: &AreaDrawParams) {
        let ctx = &draw_params.context;

        let fv_ch_path = Path::new(ctx, FillMode::Winding);
        let nf_ch_path = Path::new(ctx, FillMode::Winding);
        let wf_ch_path = Path::new(ctx, FillMode::Winding);
        let state = self.state.blocking_lock();
        if let Some(aim_point) = state.fv_aim_point {
            draw_crosshair(&ctx, &fv_ch_path, aim_point.x*draw_params.area_width, aim_point.y*draw_params.area_height, 30.);
        }
        fv_ch_path.end(ctx);
        if let Some(aim_point) = state.nf_aim_point {
            draw_crosshair(&ctx, &nf_ch_path, aim_point.x*draw_params.area_width, aim_point.y*draw_params.area_height, 15.);
        }
        nf_ch_path.end(ctx);
        if let Some(aim_point) = state.wf_aim_point {
            draw_crosshair(&ctx, &wf_ch_path, aim_point.x*draw_params.area_width, aim_point.y*draw_params.area_height, 15.);
        }
        wf_ch_path.end(ctx);

        let stroke = StrokeParams {
            cap: 0, // Bevel
            join: 0, // Flat
            thickness: 10.,
            miter_limit: 0.,
            dashes: vec![],
            dash_phase: 0.,
        };

        let brush = Brush::Solid(SolidBrush {
            r: 0.,
            g: 1.,
            b: 0.,
            a: 1.,
        });

        ctx.stroke(&fv_ch_path, &brush, &stroke);

        let stroke = StrokeParams {
            cap: 0, // Bevel
            join: 0, // Flat
            thickness: 5.,
            miter_limit: 0.,
            dashes: vec![],
            dash_phase: 0.,
        };

        let brush = Brush::Solid(SolidBrush {
            r: 1.,
            g: 0.,
            b: 0.,
            a: 1.,
        });

        ctx.stroke(&nf_ch_path, &brush, &stroke);

        let brush = Brush::Solid(SolidBrush {
            r: 0.,
            g: 0.,
            b: 1.,
            a: 1.,
        });

        ctx.stroke(&wf_ch_path, &brush, &stroke);
    }

    fn key_event(&mut self, _area: &Area, _area_key_event: &AreaKeyEvent) -> bool {
        (self.on_closing)(&mut self.window);
        true
    }
}
