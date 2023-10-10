use iui::draw;
use iui::draw::Path;

pub fn draw_crosshair(ctx: &draw::DrawContext, path: &Path, x: f64, y: f64) {
    path.new_figure(ctx, x-8., y);
    path.line_to(ctx, x+8., y);

    path.new_figure(ctx, x, y-8.);
    path.line_to(ctx, x, y+8.);
}
