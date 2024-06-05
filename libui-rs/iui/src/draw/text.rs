use std::{ffi::{CStr, CString}, ops::RangeBounds};

use ui_sys::uiDrawFreeTextLayout;

use crate::controls::FontDescription;

pub struct AttributedString {
    ui_attributed_string: *mut ui_sys::uiAttributedString,
}

impl Drop for AttributedString {
    fn drop(&mut self) {
        unsafe {
            ui_sys::uiFreeAttributedString(self.ui_attributed_string)
        }
    }
}

impl AttributedString {
    pub fn new(initial_contents: &str) -> Self {
        let initial_contents = CString::new(initial_contents).unwrap();
        Self {
            ui_attributed_string: unsafe {
                ui_sys::uiNewAttributedString(initial_contents.as_ptr())
            }
        }
    }

    pub fn len(&self) -> usize {
        unsafe {
            ui_sys::uiAttributedStringLen(self.ui_attributed_string)
        }
    }

    pub fn as_str(&self) -> &str {
        unsafe {
            let contents = ui_sys::uiAttributedStringString(self.ui_attributed_string);
            // uiAttributedString is guaranteed to be valid UTF-8
            CStr::from_ptr(contents).to_str().unwrap()
        }
    }

    /// Append the string to the end of this `AttributedString`. The new substring will be
    /// unattributed.
    pub fn push_str(&mut self, s: &str) {
        let s = CString::new(s).unwrap();
        unsafe {
            ui_sys::uiAttributedStringAppendUnattributed(self.ui_attributed_string, s.as_ptr())
        }
    }

    /// Set the color of a slice of this `AttributeString`. The RGBA values should be between 0.0
    /// and 1.0.
    pub fn color<R: RangeBounds<usize>>(&mut self, range: R, r: f64, g: f64, b: f64, a: f64) {
        let len = self.len();
        let start = match range.start_bound() {
            std::ops::Bound::Included(&n) => n,
            std::ops::Bound::Excluded(&n) => n+1,
            std::ops::Bound::Unbounded => 0,
        };
        let end = match range.end_bound() {
            std::ops::Bound::Included(&n) => n-1,
            std::ops::Bound::Excluded(&n) => n,
            std::ops::Bound::Unbounded => len,
        };
        assert!(start < len);
        assert!(end < len);
        assert!(start <= end);
        if start != end {
            unsafe {
                let attr = ui_sys::uiNewColorAttribute(r, g, b, a);
                ui_sys::uiAttributedStringSetAttribute(self.ui_attributed_string, attr, start, end)
            }
        }
    }

    pub fn layout(&self, default_font: &FontDescription, width: f64, align: TextAlign) -> Layout {
        let default_font_family = CString::new(&*default_font.family).unwrap();
        let mut default_font = ui_sys::uiFontDescriptor {
            Family: default_font_family.as_ptr() as *mut _,
            Size: default_font.size,
            Weight: default_font.weight,
            Italic: default_font.slant.as_ui_text_italic(),
            Stretch: default_font.stretch.as_ui_text_stretch(),
        };
        let mut params = ui_sys::uiDrawTextLayoutParams {
            String: self.ui_attributed_string,
            DefaultFont: &mut default_font,
            Width: width,
            Align: align.as_ui_draw_text_align(),
        };
        let layout = unsafe { ui_sys::uiDrawNewTextLayout(&mut params) };
        Layout {
            ui_draw_text_layout: layout,
        }
    }
}

pub struct Layout {
    pub(super) ui_draw_text_layout: *mut ui_sys::uiDrawTextLayout,
}

impl Drop for Layout {
    fn drop(&mut self) {
        unsafe {
            uiDrawFreeTextLayout(self.ui_draw_text_layout)
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum TextAlign {
    Left,
    Center,
    Right,
}

impl TextAlign {
    pub fn as_ui_draw_text_align(self) -> ui_sys::uiDrawTextAlign {
        match self {
            TextAlign::Left => ui_sys::uiDrawTextAlignLeft,
            TextAlign::Center => ui_sys::uiDrawTextAlignCenter,
            TextAlign::Right => ui_sys::uiDrawTextAlignRight,
        }
    }
}
