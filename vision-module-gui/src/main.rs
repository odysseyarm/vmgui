use std::ops::Deref;
use std::sync::Arc;
use iui::controls::{Button, GridAlignment, GridExpand, Group, Label, LayoutGrid, VerticalBox, Entry, HorizontalBox, Combobox, Control};
use iui::prelude::*;
use specs::prelude::*;

struct Device {
    name: String,
}

struct Update(Arc<dyn Fn()+Send+Sync>);

struct UpdateSys;

impl<'a> System<'a> for UpdateSys {
    type SystemData = ReadStorage<'a, Update>;

    fn run(&mut self, update: Self::SystemData) {
        for update in update.join() {
            update.0();
        }
    }
}

impl Component for Update {
    type Storage = VecStorage<Self>;
}

struct View<T: Into<Control>> {
    root: T,
}

fn main() {
    // Initialize the UI library
    let ui = UI::init().expect("Couldn't initialize UI library");
    // Create a window into which controls can be placed
    let mut win = Window::new(&ui, "Test App", 200, 200, WindowType::NoMenubar);

    let mut grid = LayoutGrid::new(&ui);
    grid.set_padded(&ui, true);

    let device_combobox_view = View {
        root: Combobox::new(&ui),
    };
    grid.append(
        &ui,
        device_combobox_view.root.clone(),
        0,
        0,
        1,
        1,
        GridExpand::Neither,
        GridAlignment::End,
        GridAlignment::Center,
    );
    let mut refresh_button = Button::new(&ui, "Refresh");
    grid.append(
        &ui,
        refresh_button.clone(),
        1,
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

    // let mut loading = Label::new(&ui, "Loading");

    let mut device_list: Vec<Device> = vec![];

    let mut world = World::new();
    world.register::<Update>();

    world.create_entity().with(Update(Arc::new(|| {
        // todo
        // device_combobox_view.root.clear(&ui);
        // for device in device_list {
        //     device_combobox_view.root.append(&ui, device.name.as_str());
        // }
    }))).build();

    let mut update_dispatcher = DispatcherBuilder::new().with(UpdateSys, "update_sys", &[]).build();

    refresh_button.on_clicked(&ui, move |_| {
        device_list = vec![
            Device{name: String::from("Device 1")},
            Device{name: String::from("Device 2")},
            Device{name: String::from("Device 3")},
        ];
        update_dispatcher.dispatch(&mut world);
    });

    win.set_child(&ui, grid);

    win.show(&ui);
    // Run the application
    let mut i = 0;
    let mut ev = ui.event_loop();
    ev.on_tick(&ui, || {
        println!("tick {i}");
        i += 1;
    });
    ev.run(&ui);
}
