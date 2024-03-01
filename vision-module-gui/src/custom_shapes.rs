use iui::draw;
use iui::draw::Path;
use nalgebra::{Point2, SMatrix, Transform2};

pub fn draw_crosshair(ctx: &draw::DrawContext, path: &Path, x: f64, y: f64, r: f64) {
    path.new_figure(ctx, x-r, y);
    path.line_to(ctx, x+r, y);

    path.new_figure(ctx, x, y-r);
    path.line_to(ctx, x, y+r);
}

pub fn draw_grid(ctx: &draw::DrawContext, path: &Path, x_subdiv: usize, y_subdiv: usize, transform: SMatrix<f64, 3, 3>) {
    for y in 0..=y_subdiv {
        let p1 = Point2::new(0.0, y as f64 / y_subdiv as f64);
        let p2 = Point2::new(1.0, y as f64 / y_subdiv as f64);
        let p1 = transform.transform_point(&p1);
        let p2 = transform.transform_point(&p2);
        path.new_figure(ctx, p1.x, p1.y);
        path.line_to(ctx, p2.x, p2.y);
    }

    for x in 0..=x_subdiv {
        let p1 = Point2::new(x as f64 / x_subdiv as f64, 0.0);
        let p2 = Point2::new(x as f64 / x_subdiv as f64, 1.0);
        let p1 = transform.transform_point(&p1);
        let p2 = transform.transform_point(&p2);
        path.new_figure(ctx, p1.x, p1.y);
        path.line_to(ctx, p2.x, p2.y);
    }
}

pub fn draw_diamond(ctx: &draw::DrawContext, path: &Path, x: f64, y: f64, w: f64, h: f64) {
    path.new_figure(ctx, x, y - h/2.);
    path.line_to(ctx, x + w/2., y);
    path.line_to(ctx, x, y + h/2.);
    path.line_to(ctx, x - w/2., y);
    path.close_figure(ctx);
}

/// Draw a square in the xy region \[0.5, 0.5\] transformed by `transform`.
pub fn draw_square(ctx: &draw::DrawContext, path: &Path, transform: Transform2<f64>) {
    let p1 = transform * Point2::new(-0.5, -0.5);
    let p2 = transform * Point2::new(0.5, -0.5);
    let p3 = transform * Point2::new(0.5, 0.5);
    let p4 = transform * Point2::new(-0.5, 0.5);
    path.new_figure(ctx, p1.x, p1.y);
    path.line_to(ctx, p2.x, p2.y);
    path.line_to(ctx, p3.x, p3.y);
    path.line_to(ctx, p4.x, p4.y);
    path.close_figure(ctx);
}
