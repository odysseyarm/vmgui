use std::thread;

use leptos_reactive::{create_rw_signal, SignalSet, create_effect, SignalWith, SignalGet};
use iui::controls::{Button, GridAlignment, GridExpand, Group, Label, LayoutGrid, VerticalBox, Entry, HorizontalBox, HorizontalSeparator, Combobox, Control, Spacer};
use iui::prelude::*;
use iui::controls::GridExpand::Vertical;
use iui::menus::Menu;

#[derive(Clone, Debug)]
struct Device {
    name: String,
}

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rt = tokio::runtime::Builder::new_multi_thread().enable_all().build().unwrap();
    let leptos_rt = leptos_reactive::create_runtime();
    let handle = rt.handle().clone();
    // Initialize the UI library
    let ui = UI::init().expect("Couldn't initialize UI library");

    // Create a main_window into which controls can be placed
    let mut main_win = Window::new(&ui, "ATS Vision Tool", 250, 100, WindowType::NoMenubar);

    let mut config_win = Window::new(&ui, "Config", 100, 100, WindowType::NoMenubar);
    {
        let ui2 = ui.clone();
        config_win.on_closing(&ui, move |config_win| {
            config_win.hide(&ui2);
        });
    }

    let mut vert_box = VerticalBox::new(&ui);
    vert_box.set_padded(&ui, true);

    let mut config_button = Button::new(&ui, "Config");
    let track_button = Button::new(&ui, "Start Tracking");
    let test_button = Button::new(&ui, "Run Test");

    let mut grid = LayoutGrid::new(&ui);
    grid.set_padded(&ui, false);
    grid.append(
        &ui,
        config_button.clone(),
        0,
        0,
        1,
        1,
        GridExpand::Vertical,
        GridAlignment::Fill,
        GridAlignment::Fill,
    );
    grid.append(
        &ui,
        track_button.clone(),
        1,
        0,
        1,
        1,
        GridExpand::Vertical,
        GridAlignment::Fill,
        GridAlignment::Fill,
    );
    grid.append(
        &ui,
        test_button.clone(),
        2,
        0,
        1,
        1,
        GridExpand::Vertical,
        GridAlignment::Fill,
        GridAlignment::Fill,
    );

    vert_box.append(&ui, grid.clone(), LayoutStrategy::Compact);

    vert_box.append(&ui, HorizontalSeparator::new(&ui), LayoutStrategy::Compact);
    vert_box.append(&ui, Spacer::new(&ui), LayoutStrategy::Compact);

    main_win.set_child(&ui, vert_box);

    let mut grid = LayoutGrid::new(&ui);
    grid.set_padded(&ui, true);

    let device_list = create_rw_signal(Vec::<Device>::new());

    let device_combobox = Combobox::new(&ui);
    create_effect({
        let device_combobox = device_combobox.clone();
        let ui = ui.clone();
        move |_| {
            device_combobox.clear(&ui);
            device_list.with(|device_list| {
                for device in device_list {
                    device_combobox.append(&ui, device.name.as_str());
                }
            });
        }
    });
    thread::spawn(move || {
        // don't do this
        println!("{:?}", device_list.get());
    });

    grid.append(
        &ui,
        device_combobox.clone(),
        1,
        0,
        1,
        1,
        GridExpand::Neither,
        GridAlignment::End,
        GridAlignment::Center,
    );
    let mut refresh_button = Button::new(&ui, "Refresh");
    refresh_button.on_clicked(&ui, move |_| {
        device_list.set(vec![
            Device { name: String::from("Device 1") },
            Device { name: String::from("Device 2") },
            Device { name: String::from("Device 3") },
        ]);
    });
    grid.append(
        &ui,
        refresh_button.clone(),
        2,
        0,
        1,
        1,
        GridExpand::Neither,
        GridAlignment::End,
        GridAlignment::Center,
    );
    let bank_label = Label::new(&ui, "Bank");
    grid.append(
        &ui,
        bank_label.clone(),
        0,
        1,
        1,
        1,
        GridExpand::Neither,
        GridAlignment::End,
        GridAlignment::Center,
    );
    let addr_label = Label::new(&ui, "Address");
    grid.append(
        &ui,
        addr_label.clone(),
        0,
        2,
        1,
        1,
        GridExpand::Neither,
        GridAlignment::End,
        GridAlignment::Center,
    );
    let data_label = Label::new(&ui, "Data");
    grid.append(
        &ui,
        data_label.clone(),
        0,
        3,
        1,
        1,
        GridExpand::Neither,
        GridAlignment::End,
        GridAlignment::Center,
    );

    let bank_input = Entry::new(&ui);
    grid.append(
        &ui,
        bank_input.clone(),
        1,
        1,
        1,
        1,
        GridExpand::Horizontal,
        GridAlignment::Fill,
        GridAlignment::Fill,
    );
    let addr_input = Entry::new(&ui);
    grid.append(
        &ui,
        addr_input.clone(),
        1,
        2,
        1,
        1,
        GridExpand::Horizontal,
        GridAlignment::Fill,
        GridAlignment::Fill,
    );
    let data_input = Entry::new(&ui);
    grid.append(
        &ui,
        data_input.clone(),
        1,
        3,
        1,
        1,
        GridExpand::Horizontal,
        GridAlignment::Fill,
        GridAlignment::Fill,
    );

    let mut buttons_hbox = HorizontalBox::new(&ui);
    buttons_hbox.set_padded(&ui, true);
    grid.append(&ui, buttons_hbox.clone(), 0, 4, 2, 1,
        GridExpand::Horizontal,
        GridAlignment::Fill,
        GridAlignment::Fill,
    );
    let read_button = Button::new(&ui, "Read");
    buttons_hbox.append(&ui, read_button.clone(), LayoutStrategy::Stretchy);
    let write_button = Button::new(&ui, "Write");
    buttons_hbox.append(&ui, write_button.clone(), LayoutStrategy::Stretchy);

    config_win.set_child(&ui, grid);
    {
        let ui2 = ui.clone();
        config_button.on_clicked(&ui, move |_| {
            config_win.show(&ui2);
        });
    }

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
