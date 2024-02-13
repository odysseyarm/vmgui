/// Creates layout code from a compact, declarative and hierarchical UI description.
///
/// # Example
///
/// For a more example, see the [builder example application](https://github.com/iui-rs/libui/tree/development/libui/examples) in the repository.
///
/// ```no_run
/// extern crate iui;
/// use iui::prelude::*;
///
/// fn main() {
///     let ui = UI::init().unwrap();
///
///     iui::layout! { &ui,
///         let layout = VerticalBox(padded: true) {
///             Compact: let form = Form(padded: true) {
///                 (Compact, "User"): let tb_user = Entry()
///                 (Compact, "Password"): let tb_passwd = Entry()
///             }
///             Stretchy: let bt_submit = Button("Submit")
///         }
///     }
///
///     let mut window = Window::new(&ui, "Builder Example", 320, 200,
///         WindowType::NoMenubar);
///
///     window.set_child(&ui, layout);
///     window.show(&ui);
///     ui.main();
/// }
/// ```
#[macro_export]
macro_rules! layout {

    // ---------------------- Controls without children -----------------------

    // Button
    [ $ui:expr ,
        let $ctl:ident = Button ( $text:expr )
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::Button::new($ui, $text);
    ];

    // Checkbox
    [ $ui:expr ,
        let $ctl:ident = Checkbox ( $text:expr $( , checked: $checked:expr )? )
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::Checkbox::new($ui, $text);
        $( $ctl.set_checked($ui, $checked); )?
    ];

    // ColorButton
    [ $ui:expr ,
        let $ctl:ident = ColorButton ()
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::ColorButton::new($ui);
    ];

    // Combobox
    [ $ui:expr ,
        let $ctl:ident = Combobox ( $( selected: $selected:expr )? )
        { $( $option:expr ),* }
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::Combobox::new($ui);
        $( $ctl.append($ui, $option); )*
        $( $ctl.set_selected($ui, $selected); )?
    ];

    // DateTimePicker
    [ $ui:expr ,
        let $ctl:ident = DateTimePicker ( $kind:ident )
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::DateTimePicker::new($ui,
            iui::controls::DateTimePickerKind::$kind);
    ];

    // EditableCombobox
    [ $ui:expr ,
        let $ctl:ident = EditableCombobox ()
        { $( $option:expr ),* }
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::EditableCombobox::new($ui);
        $( $ctl.append($ui, $option); )*
    ];

    // Entry
    [ $ui:expr ,
        let $ctl:ident = Entry ()
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::Entry::new($ui);
    ];

    // FontButton
    [ $ui:expr ,
        let $ctl:ident = FontButton ()
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::FontButton::new($ui);
    ];

    // HorizontalSeparator
    [ $ui:expr ,
        let $ctl:ident = HorizontalSeparator ()
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::HorizontalSeparator::new($ui);
    ];

    // Label
    [ $ui:expr ,
        let $ctl:ident = Label ( $text:expr )
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::Label::new($ui, $text);
    ];

    // MultilineEntry
    [ $ui:expr ,
        let $ctl:ident = MultilineEntry ()
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::MultilineEntry::new($ui);
    ];

    // MultilineEntry, wrapping option
    [ $ui:expr ,
        let $ctl:ident = MultilineEntry ( wrapping: $wrapping:expr )
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = if $wrapping {
            iui::controls::MultilineEntry::new($ui)
        } else {
            iui::controls::MultilineEntry::new_nonwrapping($ui)
        };
    ];

    // PasswordEntry
    [ $ui:expr ,
        let $ctl:ident = PasswordEntry ()
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::PasswordEntry::new($ui);
    ];

    // RadioButtons
    [ $ui:expr ,
        let $ctl:ident = RadioButtons ( $( selected: $selected:expr )? )
        { $( $option:expr ),* }
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::RadioButtons::new($ui);
        $( $ctl.append($ui, $option); )*
        $( $ctl.set_selected($ui, $selected); )?
    ];

    // SearchEntry
    [ $ui:expr ,
        let $ctl:ident = SearchEntry ()
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::SearchEntry::new($ui);
    ];

    // Slider
    [ $ui:expr ,
        let $ctl:ident = Slider ( $min:expr , $max:expr )
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::Slider::new($ui, $min, $max);
    ];

    // Spacer
    [ $ui:expr ,
        let $ctl:ident = Spacer ()
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::Spacer::new($ui);
    ];

    // Spinbox, limited
    [ $ui:expr ,
        let $ctl:ident = Spinbox ( $min:expr , $max:expr )
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::Spinbox::new($ui, $min, $max);
    ];

    // Spinbox, unlimited
    [ $ui:expr ,
        let $ctl:ident = Spinbox ()
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::Spinbox::new_unlimited($ui);
    ];

    // ProgressBar
    [ $ui:expr ,
        let $ctl:ident = ProgressBar ()
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::ProgressBar::new();
    ];

    // ----------------- Controls with children (Containers) ------------------

    // Form
    [ $ui:expr ,
        let $ctl:ident = Form ( $( padded: $padded:expr )? )
        { $(
            ( $strategy:ident, $name:expr ) :
            let $child:ident = $type:ident ($($opt:tt)*) $({$($body:tt)*})?
        )* }
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::Form::new($ui);
        $( $ctl.set_padded($ui, $padded); )?
        $(
            iui::layout! { $ui, let $child = $type ($($opt)*) $({$($body)*})? }
            $ctl.append($ui, $name, $child.clone(),
                    iui::controls::LayoutStrategy::$strategy);
        )*
    ];

    // Group
    [ $ui:expr ,
        let $ctl:ident = Group ( $title:expr $( , margined: $margined:expr )? )
        { $(
            let $child:ident = $type:ident ($($opt:tt)*) $({$($body:tt)*})?
        )? }
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::Group::new($ui, $title);
        $( $ctl.set_margined($margined); )?
        $(
            iui::layout! { $ui, let $child = $type ($($opt)*) $({$($body)*})? }
            $ctl.set_child($ui, $child.clone());
        )?
    ];

    // HorizontalBox
    [ $ui:expr ,
        let $ctl:ident = HorizontalBox ( $( padded: $padded:expr )? )
        { $(
            $strategy:ident :
            let $child:ident = $type:ident ($($opt:tt)*) $({$($body:tt)*})?
        )* }
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::HorizontalBox::new($ui);
        $( $ctl.set_padded($ui, $padded); )?
        $(
            iui::layout! { $ui, let $child = $type ($($opt)*) $({$($body)*})? }
            $ctl.append($ui, $child.clone(),
                        iui::controls::LayoutStrategy::$strategy);
        )*
    ];

    // LayoutGrid
    [ $ui:expr ,
        let $ctl:ident = LayoutGrid ( $( padded: $padded:expr )? )
        { $(
            ( $x:expr , $y:expr ) ( $xspan:expr , $yspan:expr )
            $expand:ident ( $halign:ident , $valign:ident ) :
            let $child:ident = $type:ident ($($opt:tt)*) $({$($body:tt)*})?
        )* }
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::LayoutGrid::new($ui);
        $( $ctl.set_padded($ui, $padded); )?
        $(
            iui::layout! { $ui, let $child = $type ($($opt)*) $({$($body)*})? }
            $ctl.append($ui, $child.clone(), $x, $y, $xspan, $yspan,
                        iui::controls::GridExpand::$expand,
                        iui::controls::GridAlignment::$halign,
                        iui::controls::GridAlignment::$valign);
        )*
    ];

    // TabGroup
    [ $ui:expr ,
        let $ctl:ident = TabGroup ()
        { $(
            ( $name:expr $( , margined: $margined:expr )? ) :
            let $child:ident = $type:ident ($($opt:tt)*) $({$($body:tt)*})?
        )* }
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::TabGroup::new($ui);
        $(
            iui::layout! { $ui, let $child = $type ($($opt)*) $({$($body)*})? }
            let __tab_n = $ctl.append($ui, $name, $child.clone());
            $( $ctl.set_margined($ui, __tab_n - 1, $margined); )?
        )*
    ];

    // VerticalBox
    [ $ui:expr ,
        let $ctl:ident = VerticalBox ( $( padded: $padded:expr )? )
        { $(
            $strategy:ident :
            let $child:ident = $type:ident ($($opt:tt)*) $({$($body:tt)*})?
        )* }
    ] => [
        #[allow(unused_mut)]
        let mut $ctl = iui::controls::VerticalBox::new($ui);
        $( $ctl.set_padded($ui, $padded); )?
        $(
            iui::layout! { $ui, let $child = $type ($($opt)*) $({$($body)*})? }
            $ctl.append($ui, $child.clone(),
                        iui::controls::LayoutStrategy::$strategy);
        )*
    ];
}

/// Creates menu entries for the applications main menu from a hierarchical description.
///
/// # Example
///
/// ```no_run
/// extern crate iui;
/// use iui::prelude::*;
///
/// fn main() {
///     let ui = UI::init().unwrap();
///
///     iui::menu! { &ui,
///         let menu_file = Menu("File") {
///             let menu_file_open = MenuItem("Open")
///             let menu_file_save = MenuItem("Save")
///             let menu_file_close = MenuItem("Close")
///             Separator()
///             let menu_file_quit = MenuItem("Exit")
///         }
///         let menu_settings = Menu("Settings") {
///             let menu_settings_num = MenuItem("Line Numbers", checked: true)
///         }
///         let menu_help = Menu("Help") {
///             let menu_help_about = MenuItem("About")
///         }
///     }
///
///     let mut window = Window::new(&ui, "Title", 300, 200, WindowType::HasMenubar);
///     iui::layout! { &ui,
///         let layout = VerticalBox() { }
///     }
///     window.set_child(&ui, layout);
///     window.show(&ui);
///     ui.main();
/// }
/// ```
#[macro_export]
macro_rules! menu {

    // End recursion
    [@impl $parent:ident,] => [];

    // MenuItem
    [@impl $parent:ident,
        let $item:ident = MenuItem ( $text:expr )
        $($tail:tt)*
    ] => [
        #[allow(unused_mut)]
        let mut $item = $parent.append_item($ui, $text);
        iui::menu! { @impl $parent, $($tail)* }
    ];

    // Checked MenuItem
    [@impl $parent:ident,
        let $item:ident = MenuItem ( $text:expr, checked: $checked:expr )
        $($tail:tt)*
    ] => [
        #[allow(unused_mut)]
        let mut $item = $parent.append_check_item($ui, $text);
        $item.set_checked($checked);
        iui::menu! { @impl $parent, $($tail)* }
    ];

    // Separator
    [@impl $parent:ident,
        Separator ( )
        $($tail:tt)*
    ] => [
        $parent.append_separator($ui);
        iui::menu! { @impl $parent, $($tail)* }
    ];

    // Menu
    [ $ui:expr ,
        $(
            let $menu:ident = Menu ( $name:expr )
            {
                $($tail:tt)*
            }
        )+
    ] => [
        $(
            #[allow(unused_mut)]
            let mut $menu = iui::menus::Menu::new( $name );
            iui::menu! { @impl $menu, $($tail)* }
        )+
    ];
}
