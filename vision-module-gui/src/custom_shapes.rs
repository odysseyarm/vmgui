use iui::draw;
use iui::draw::Path;

pub fn draw_crosshair(ctx: &draw::DrawContext, path: &Path, x: f64, y: f64, r: f64) {
    path.new_figure(ctx, x-r, y);
    path.line_to(ctx, x+r, y);

    path.new_figure(ctx, x, y-r);
    path.line_to(ctx, x, y+r);
}
