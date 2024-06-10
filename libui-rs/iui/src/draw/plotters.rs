use plotters_backend::text_anchor::{HPos, VPos};
use plotters_backend::{
    BackendColor, BackendCoord, BackendStyle, BackendTextStyle, DrawingBackend, DrawingErrorKind,
    FontStyle, FontTransform,
};

use crate::controls::{AreaDrawParams, FontDescription, SlantStyle, StretchStyle};

use super::{
    text::{AttributedString, TextAlign},
    Brush, DrawContext, FillMode, Path, SolidBrush, StrokeParams, Transform,
};

/// The drawing backend that is backed with a Cairo context
pub struct PlottersBackend<'a> {
    context: &'a DrawContext,
    draw_params: &'a AreaDrawParams,
    width: u32,
    height: u32,
    init_flag: bool,
}

impl<'a> PlottersBackend<'a> {
    pub fn new(draw_params: &'a AreaDrawParams, (w, h): (u32, u32)) -> Self {
        Self {
            draw_params,
            context: &draw_params.context,
            width: w,
            height: h,
            init_flag: false,
        }
    }
}

#[derive(Debug)]
pub struct Error;

impl std::fmt::Display for Error {
    fn fmt(&self, _f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        Ok(())
    }
}

impl std::error::Error for Error {}

impl<'a> DrawingBackend for PlottersBackend<'a> {
    type ErrorType = Error;

    fn get_size(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    fn ensure_prepared(&mut self) -> Result<(), DrawingErrorKind<Self::ErrorType>> {
        if !self.init_flag {
            let AreaDrawParams {
                area_width: w,
                area_height: h,
                // clip_x: x,
                // clip_y: y,
                // clip_width: w,
                // clip_height: h,
                ..
            } = *self.draw_params;

            let mut transform = Transform::identity();
            transform.scale(0.0, 0.0, w / self.width as f64, h / self.height as f64);
            // transform.translate(x, y);
            self.context.transform(&transform);

            self.init_flag = true;
        }

        Ok(())
    }

    fn present(&mut self) -> Result<(), DrawingErrorKind<Self::ErrorType>> {
        Ok(())
    }

    fn draw_pixel(
        &mut self,
        point: BackendCoord,
        color: BackendColor,
    ) -> Result<(), DrawingErrorKind<Self::ErrorType>> {
        let ctx = self.context;
        let path = Path::new(ctx, FillMode::Winding);
        path.add_rectangle(ctx, point.0 as f64, point.1 as f64, 1.0, 1.0);
        path.end(ctx);
        ctx.fill(&path, &color_to_solid_brush(color));
        Ok(())
    }

    fn draw_line<S: BackendStyle>(
        &mut self,
        from: BackendCoord,
        to: BackendCoord,
        style: &S,
    ) -> Result<(), DrawingErrorKind<Self::ErrorType>> {
        let ctx = self.context;
        let path = Path::new(ctx, FillMode::Winding);
        path.new_figure(ctx, from.0 as f64, from.1 as f64);
        path.line_to(ctx, to.0 as f64, to.1 as f64);
        path.end(ctx);
        ctx.stroke(
            &path,
            &color_to_solid_brush(style.color()),
            &stroke_params(style),
        );
        Ok(())
    }

    fn draw_rect<S: BackendStyle>(
        &mut self,
        upper_left: BackendCoord,
        bottom_right: BackendCoord,
        style: &S,
        fill: bool,
    ) -> Result<(), DrawingErrorKind<Self::ErrorType>> {
        let ctx = self.context;
        let path = Path::new(ctx, FillMode::Winding);
        path.add_rectangle(
            ctx,
            upper_left.0 as f64,
            upper_left.1 as f64,
            (bottom_right.0 - upper_left.0) as f64,
            (bottom_right.1 - upper_left.1) as f64,
        );
        path.end(ctx);

        if fill {
            ctx.fill(&path, &color_to_solid_brush(style.color()));
        } else {
            ctx.stroke(
                &path,
                &color_to_solid_brush(style.color()),
                &stroke_params(style),
            );
        }
        Ok(())
    }

    fn draw_path<S: BackendStyle, I: IntoIterator<Item = BackendCoord>>(
        &mut self,
        path: I,
        style: &S,
    ) -> Result<(), DrawingErrorKind<Self::ErrorType>> {
        let mut points = path.into_iter();
        let ctx = self.context;
        let path = Path::new(ctx, FillMode::Winding);

        if let Some((x, y)) = points.next() {
            path.new_figure(ctx, x as f64, y as f64);
        }

        for (x, y) in points {
            path.line_to(ctx, x as f64, y as f64);
        }
        path.end(ctx);
        ctx.stroke(
            &path,
            &color_to_solid_brush(style.color()),
            &stroke_params(style)
        );
        Ok(())
    }

    fn fill_polygon<S: BackendStyle, I: IntoIterator<Item = BackendCoord>>(
        &mut self,
        path: I,
        style: &S,
    ) -> Result<(), DrawingErrorKind<Self::ErrorType>> {
        let mut points = path.into_iter();
        let ctx = self.context;
        let path = Path::new(ctx, FillMode::Winding);

        if let Some((x, y)) = points.next() {
            path.new_figure(ctx, x as f64, y as f64);
        }

        for (x, y) in points {
            path.line_to(ctx, x as f64, y as f64);
        }
        path.end(ctx);
        ctx.fill(&path, &color_to_solid_brush(style.color()));
        Ok(())
    }

    fn draw_circle<S: BackendStyle>(
        &mut self,
        center: BackendCoord,
        radius: u32,
        style: &S,
        fill: bool,
    ) -> Result<(), DrawingErrorKind<Self::ErrorType>> {
        let ctx = self.context;
        let path = Path::new(ctx, FillMode::Winding);
        path.new_figure_with_arc(
            ctx,
            center.0 as f64,
            center.1 as f64,
            radius as f64,
            0.0,
            std::f64::consts::TAU,
            false,
        );
        path.end(ctx);
        if fill {
            ctx.fill(&path, &color_to_solid_brush(style.color()));
        } else {
            ctx.stroke(
                &path,
                &color_to_solid_brush(style.color()),
                &stroke_params(style),
            );
        }
        Ok(())
    }

    fn estimate_text_size<S: BackendTextStyle>(
        &self,
        text: &str,
        font: &S,
    ) -> Result<(u32, u32), DrawingErrorKind<Self::ErrorType>> {
        let attr_str = AttributedString::new(text);
        let layout = attr_str.layout(&font_description(font), -1.0, TextAlign::Left);
        let (w, h) = layout.extents();
        Ok((w as u32, h as u32))
    }

    fn draw_text<S: BackendTextStyle>(
        &mut self,
        text: &str,
        style: &S,
        pos: BackendCoord,
    ) -> Result<(), DrawingErrorKind<Self::ErrorType>> {
        let ctx = self.context;
        let color = style.color();
        let (x, y) = (pos.0, pos.1);

        let angle = match style.transform() {
            FontTransform::None => 0.0,
            FontTransform::Rotate90 => 90.0,
            FontTransform::Rotate180 => 180.0,
            FontTransform::Rotate270 => 270.0,
            //FontTransform::RotateAngle(angle) => angle as f64,
        } / 180.0
            * std::f64::consts::PI;

        if angle != 0.0 {
            let mut transform = Transform::identity();
            transform.rotate(angle, x as f64, y as f64);
        }

        let mut attr_str = AttributedString::new(text);
        attr_str.color(
            ..,
            color.rgb.0 as f64 / 255.0,
            color.rgb.1 as f64 / 255.0,
            color.rgb.2 as f64 / 255.0,
            color.alpha,
        );
        let mut layout = attr_str.layout(&font_description(style), -1.0, TextAlign::Left);
        let (text_width, text_height) = layout.extents();

        let dx = match style.anchor().h_pos {
            HPos::Left => 0.0,
            HPos::Right => -text_width,
            HPos::Center => -text_width / 2.0,
        };
        let dy = match style.anchor().v_pos {
            VPos::Top => text_height,
            VPos::Center => text_height / 2.0,
            VPos::Bottom => 0.0,
        };

        ctx.draw_text(&mut layout, x as f64 + dx, y as f64 + dy - text_height);

        if angle != 0.0 {
            let mut transform = Transform::identity();
            transform.rotate(-angle, x as f64, y as f64);
        }

        Ok(())
    }
}

fn font_description<S: BackendTextStyle>(font: &S) -> FontDescription {
    let (slant, weight) = match font.style() {
        FontStyle::Normal => (SlantStyle::Normal, 400),
        FontStyle::Bold => (SlantStyle::Normal, 700),
        FontStyle::Oblique => (SlantStyle::Oblique, 400),
        FontStyle::Italic => (SlantStyle::Italic, 400),
    };
    FontDescription {
        family: font.family().as_str().into(),
        slant,
        weight,
        size: font.size(),
        stretch: StretchStyle::Normal,
    }
}

fn color_to_solid_brush(color: BackendColor) -> Brush {
    Brush::Solid(SolidBrush {
        r: color.rgb.0 as f64 / 255.0,
        g: color.rgb.1 as f64 / 255.0,
        b: color.rgb.2 as f64 / 255.0,
        a: color.alpha,
    })
}

fn stroke_params<S: BackendStyle>(style: &S) -> StrokeParams {
    StrokeParams {
        thickness: style.stroke_width() as f64,
        cap: 2,  // Square
        join: 0, // Flat
        miter_limit: 0.,
        dashes: vec![],
        dash_phase: 0.,
    }
}
