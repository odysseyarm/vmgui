use std::{ffi::{CStr, CString}, ops::RangeBounds};

use ui_sys::{uiDrawFreeTextLayout, uiDrawTextLayoutExtents};

use crate::controls::FontDescription;

/// Represents a string of text that can
/// optionally be embellished with formatting attributes. libui
/// provides the list of formatting attributes, which cover common
/// formatting traits like boldface and color as well as advanced
/// typographical features provided by OpenType like superscripts
/// and small caps. These attributes can be combined in a variety of
/// ways.
///
/// Attributes are applied to runs of Unicode codepoints in the string.
/// Zero-length runs are elided. Consecutive runs that have the same
/// attribute type and value are merged. Each attribute is independent
/// of each other attribute; overlapping attributes of different types
/// do not split each other apart, but different values of the same
/// attribute type do.
///
/// `AttributedString` does not provide enough information to be able
/// to draw itself onto a [`DrawContext`](`::draw::DrawContext`) or respond to user actions.
/// In order to do that, you'll need to use a [`Layout`], which
/// is built using the [`layout`](`AttributedString::layout`) method.
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
    /// Create a new `AttributedString`.
    pub fn new(initial_contents: &str) -> Self {
        let initial_contents = CString::new(initial_contents).unwrap();
        Self {
            ui_attributed_string: unsafe {
                ui_sys::uiNewAttributedString(initial_contents.as_ptr())
            }
        }
    }

    /// Returns the length of `self` in bytes.
    pub fn len(&self) -> usize {
        unsafe {
            ui_sys::uiAttributedStringLen(self.ui_attributed_string)
        }
    }

    /// Extracts a string slice of the text content of `self`.
    pub fn as_str(&self) -> &str {
        unsafe {
            let contents = ui_sys::uiAttributedStringString(self.ui_attributed_string);
            // uiAttributedString is guaranteed to be valid UTF-8
            CStr::from_ptr(contents).to_str().unwrap()
        }
    }

    /// Appends a string to the end of this `AttributedString`. The new substring will be
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

    /// Creates a [`Layout`] which can be passed to
    /// [`DrawContext::draw_text`](`::draw::DrawContext::draw_text`).
    ///
    /// `default_font` is used to render any text that is not attributed sufficiently.
    ///
    /// `width` determines the width of the bounding box of the text. The height is determined
    /// automatically.
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

/// `Layout` is a concrete representation of an
/// [`AttributedString`] that can be displayed in a [`DrawContext`](`::draw::DrawContext`).
/// It includes information important for the drawing of a block of
/// text, including the bounding box to wrap the text within, the
/// alignment of lines of text within that box, areas to mark as
/// being selected, and other things.
///
/// Unlike [`AttributedString`], the content of a `Layout` is
/// immutable once it has been created.
///
/// This `struct` is created by the [`AttributedString::layout`] function.
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

impl Layout {
    /// Returns the actual width and height of the text.
    pub fn extents(&self) -> (f64, f64) {
        let mut width = 0.0;
        let mut height = 0.0;
        unsafe {
            uiDrawTextLayoutExtents(self.ui_draw_text_layout, &mut width, &mut height);
        }
        (width, height)
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
            TextAlign::Left => ui_sys::uiDrawTextAlignLeft as _,
            TextAlign::Center => ui_sys::uiDrawTextAlignCenter as _,
            TextAlign::Right => ui_sys::uiDrawTextAlignRight as _,
        }
    }
}
