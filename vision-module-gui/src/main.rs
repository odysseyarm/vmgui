use std::thread;

use iui::controls::{
    Button, Combobox, Control, Entry, GridAlignment as GA, GridExpand as GE, Group, HorizontalBox,
    HorizontalSeparator, Label, LayoutGrid, Spacer, VerticalBox,
};
use iui::menus::Menu;
use iui::prelude::*;
use leptos_reactive::{create_effect, create_rw_signal, SignalGet, SignalSet, SignalWith};

#[derive(Clone, Debug)]
struct Device {
    name: String,
}

trait CloneButShorter: Clone {
    /// Use mainly for GUI code.
    fn c(&self) -> Self {
        self.clone()
    }
}

impl<T: Clone> CloneButShorter for T {}

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    // let rt = tokio::runtime::Builder::new_multi_thread().enable_all().build().unwrap();
    let leptos_rt = leptos_reactive::create_runtime();
    // Initialize the UI library
    let ui = UI::init().expect("Couldn't initialize UI library");

    // Create a main_window into which controls can be placed
    let mut main_win = Window::new(&ui, "ATS Vision Tool", 250, 100, WindowType::NoMenubar);

    let mut config_win = Window::new(&ui, "Config", 100, 100, WindowType::NoMenubar);
    config_win.on_closing(&ui, {
        let ui = ui.c();
        move |config_win| {
            config_win.hide(&ui);
        }
    });

    let mut vert_box = VerticalBox::new(&ui);
    vert_box.set_padded(&ui, true);

    let mut config_button = Button::new(&ui, "Config");
    let track_button = Button::new(&ui, "Start Tracking");
    let test_button = Button::new(&ui, "Run Test");

    let mut grid = LayoutGrid::new(&ui);
    grid.set_padded(&ui, false);
    grid.append(&ui, config_button.c(), 0, 0, 1, 1, GE::Vertical, GA::Fill, GA::Fill);
    grid.append(&ui, track_button.c(), 1, 0, 1, 1, GE::Vertical, GA::Fill, GA::Fill);
    grid.append(&ui, test_button.c(), 2, 0, 1, 1, GE::Vertical, GA::Fill, GA::Fill);

    vert_box.append(&ui, grid.c(), LayoutStrategy::Compact);
    vert_box.append(&ui, HorizontalSeparator::new(&ui), LayoutStrategy::Compact);
    vert_box.append(&ui, Spacer::new(&ui), LayoutStrategy::Compact);

    main_win.set_child(&ui, vert_box);

    let mut grid = LayoutGrid::new(&ui);
    grid.set_padded(&ui, true);

    let device_list = create_rw_signal(Vec::<Device>::new());

    let device_combobox = Combobox::new(&ui);
    create_effect({
        let device_combobox = device_combobox.c();
        let ui = ui.c();
        move |_| {
            device_combobox.clear(&ui);
            device_list.with(|device_list| {
                for device in device_list {
                    device_combobox.append(&ui, device.name.as_str());
                }
            });
        }
    });
    grid.append(&ui, device_combobox.c(), 1, 0, 1, 1, GE::Neither, GA::End, GA::Center);

    let mut refresh_button = Button::new(&ui, "Refresh");
    refresh_button.on_clicked(&ui, move |_| {
        device_list.set(vec![
            Device {
                name: String::from("Device 1"),
            },
            Device {
                name: String::from("Device 2"),
            },
            Device {
                name: String::from("Device 3"),
            },
        ]);
    });
    grid.append(&ui, refresh_button.c(), 2, 0, 1, 1, GE::Neither, GA::End, GA::Center);
    let bank_label = Label::new(&ui, "Bank");
    grid.append(&ui, bank_label.c(), 0, 1, 1, 1, GE::Neither, GA::End, GA::Center);
    let addr_label = Label::new(&ui, "Address");
    grid.append(&ui, addr_label.c(), 0, 2, 1, 1, GE::Neither, GA::End, GA::Center);
    let data_label = Label::new(&ui, "Data");
    grid.append(&ui, data_label.c(), 0, 3, 1, 1, GE::Neither, GA::End, GA::Center);

    let bank_input = Entry::new(&ui);
    grid.append(&ui, bank_input.c(), 1, 1, 1, 1, GE::Horizontal, GA::Fill, GA::Fill);
    let addr_input = Entry::new(&ui);
    grid.append(&ui, addr_input.c(), 1, 2, 1, 1, GE::Horizontal, GA::Fill, GA::Fill);
    let data_input = Entry::new(&ui);
    grid.append(&ui, data_input.c(), 1, 3, 1, 1, GE::Horizontal, GA::Fill, GA::Fill);

    let mut buttons_hbox = HorizontalBox::new(&ui);
    buttons_hbox.set_padded(&ui, true);
    grid.append(&ui, buttons_hbox.c(), 0, 4, 2, 1, GE::Horizontal, GA::Fill, GA::Fill);
    let read_button = Button::new(&ui, "Read");
    buttons_hbox.append(&ui, read_button.c(), LayoutStrategy::Stretchy);
    let write_button = Button::new(&ui, "Write");
    buttons_hbox.append(&ui, write_button.c(), LayoutStrategy::Stretchy);

    config_win.set_child(&ui, grid);
    config_button.on_clicked(&ui, {
        let ui = ui.c();
        move |_| {
            config_win.show(&ui);
        }
    });

    main_win.show(&ui);

    // Run the application
    let mut i = 0;
    let mut ev = ui.event_loop();
    ev.on_tick(&ui, || {
        println!("tick {i}");
        i += 1;
    });
    ev.run(&ui);

    leptos_rt.dispose();
    Ok(())
}
