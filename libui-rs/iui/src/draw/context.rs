use std::ffi::CString;
use draw::{Brush, Path, StrokeParams, Transform};
use ui_sys::{self, uiDrawContext, uiDrawNewTextLayout, uiDrawTextLayoutParams, uiFontDescriptor, uiNewAttributedString};

/// Drawing context, used to draw custom content on the screen.
pub struct DrawContext {
    ui_draw_context: *mut uiDrawContext,
}

impl DrawContext {
    /// Create a Context from a ui_draw_context pointer.
    ///
    /// # Unsafety
    /// If the pointer is invalid, this is memory-unsafe.
    /// If libui is not initialized, behavior will be inconsistent.
    pub unsafe fn from_ui_draw_context(ui_draw_context: *mut uiDrawContext) -> DrawContext {
        DrawContext {
            ui_draw_context: ui_draw_context,
        }
    }

    /// Draw a stroke on this DrawContext which runs along the given Path, with the given Brush and StrokeParams.
    pub fn stroke(&self, path: &Path, brush: &Brush, stroke_params: &StrokeParams) {
        unsafe {
            let brush = brush.as_ui_draw_brush_ref(self);
            let stroke_params = stroke_params.as_stroke_params_ref(self);
            ui_sys::uiDrawStroke(
                self.ui_draw_context,
                path.ptr(),
                brush.ptr(),
                stroke_params.ptr(),
            )
        }
    }

    /// Draw a fill on this DrawContext using the given Path using the given Brush.
    pub fn fill(&self, path: &Path, brush: &Brush) {
        unsafe {
            let brush = brush.as_ui_draw_brush_ref(self);
            ui_sys::uiDrawFill(self.ui_draw_context, path.ptr(), brush.ptr())
        }
    }

    /// Transform this DrawContext by the given Transform.
    pub fn transform(&self, txform: &Transform) {
        unsafe { ui_sys::uiDrawTransform(self.ui_draw_context, txform.ptr()) }
    }

    /// Open a modal allowing the user to save the contents of this DrawContext.
    pub fn save(&self) {
        unsafe { ui_sys::uiDrawSave(self.ui_draw_context) }
    }

    /// Open a modal allowing the user to load the contents of a DrawContext onto this one.
    pub fn restore(&self) {
        unsafe { ui_sys::uiDrawRestore(self.ui_draw_context) }
    }

    // pub fn draw_text(&self, tl: &mut uiDrawTextLayout, x: f64, y: f64) {
    //     unsafe {
    //         ui_sys::uiDrawText(self.ui_draw_context, tl, x, y)
    //     }
    // }

    pub fn draw_text(&self, x: f64, y: f64, text: &str) {
        unsafe {
            let c_string = CString::new(text.as_bytes().to_vec()).unwrap();
            let string = uiNewAttributedString(c_string.as_ptr());
            let family = CString::new("Courier New".as_bytes().to_vec()).unwrap();
            let mut font = uiFontDescriptor {
                Family: family.as_ptr().cast_mut(),
                Size: 12.0,
                Weight: 1,
                Italic: 0,
                Stretch: 0,
            };
            let mut params = uiDrawTextLayoutParams {
                String: string,
                DefaultFont: &mut font,
                Width: 200.0,
                Align: 0,
            };
            let layout = uiDrawNewTextLayout(&mut params);
            ui_sys::uiDrawText(self.ui_draw_context, layout, x, y);
            ui_sys::uiFreeAttributedString(string);
            ui_sys::uiDrawFreeTextLayout(layout);
        }
    }
}
