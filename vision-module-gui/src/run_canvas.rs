use crate::mot_runner::MotRunner;
use crate::{tracking_canvas_helpers, CloneButShorter};
use iui::controls::{Area, AreaDrawParams, AreaHandler};
use iui::UI;
use parking_lot::Mutex;
use std::sync::Arc;

pub struct RunCanvas {
    pub ctx: UI,
    pub runner: Arc<Mutex<MotRunner>>,
}

impl AreaHandler for RunCanvas {
    fn draw(&mut self, _area: &Area, draw_params: &AreaDrawParams) {
        tracking_canvas_helpers::draw(self.ctx.c(), self.runner.c(), _area, draw_params, false);
    }
}
