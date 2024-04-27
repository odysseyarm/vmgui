use std::sync::Arc;
use parking_lot::Mutex;
use iui::controls::{Area, AreaDrawParams, AreaHandler};
use iui::UI;
use crate::mot_runner::MotRunner;
use crate::{tracking_canvas_helpers, CloneButShorter};

pub struct RunRawCanvas {
    pub ctx: UI,
    pub runner: Arc<Mutex<MotRunner>>,
}

impl AreaHandler for RunRawCanvas {
    fn draw(&mut self, _area: &Area, draw_params: &AreaDrawParams) {
        tracking_canvas_helpers::draw(self.ctx.c(), self.runner.c(), _area, draw_params, true);
    }
}
