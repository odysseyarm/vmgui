use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;
use tokio::sync::mpsc::{Sender, Receiver};
use tokio::runtime::Handle;
use iui::controls::{Button, GridAlignment, GridExpand, Group, Label, LayoutGrid, VerticalBox, Entry, HorizontalBox, HorizontalSeparator, Combobox, Control, Spacer};
use iui::prelude::*;
use specs::prelude::*;
use iui::controls::GridExpand::Vertical;
use iui::menus::Menu;

struct Device {
    name: String,
}

struct Update(Sender<()>);

struct UpdateSys {
    handle: Handle,
}

impl<'a> System<'a> for UpdateSys {
    type SystemData = ReadStorage<'a, Update>;

    fn run(&mut self, update: Self::SystemData) {
        for update in (&update).join() {
            let tx = update.0.clone();
            self.handle.spawn(async move { tx.send(()).await });
        }
    }
}

impl Component for Update {
    type Storage = VecStorage<Self>;
}

struct Controller {
    channel: (Sender<()>, Receiver<()>),
    update: Box<dyn Fn()>,
}

impl Controller {
    async fn run(&mut self) {
        loop {
            match self.channel.1.recv().await {
                Some(()) => { (*(self.update))(); },
                None => todo!(),
            }
        }
    }
}

#[tokio::main]
pub async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize the UI library
    let ui = UI::init().expect("Couldn't initialize UI library");

    // Create a window into which controls can be placed
    let mut win = Window::new(&ui, "Test App", 200, 200, WindowType::NoMenubar);
    win.set_margined(&ui, false);

    let mut vert_box = VerticalBox::new(&ui);
    vert_box.set_padded(&ui, false);

    let btn = Button::new(&ui, "");

    let button_1 = Button::new(&ui, "Menu Button 1");
    let button_2 = Button::new(&ui, "Menu Button 2");

    let mut grid = LayoutGrid::new(&ui);
    grid.set_padded(&ui, false);
    grid.append(
        &ui,
        button_1.clone(),
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
        button_2.clone(),
        1,
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

    let mut grid = LayoutGrid::new(&ui);
    grid.set_padded(&ui, true);

    vert_box.append(&ui, grid.clone(), LayoutStrategy::Stretchy);

    let device_list = Arc::new(Mutex::new(Vec::<Device>::new()));

    let device_combobox = Combobox::new(&ui);

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

    // let mut loading = Label::new(&ui, "Loading");

    let mut world = World::new();
    world.register::<Update>();

    {
        let ui2 = ui.clone();
        let device_combobox = device_combobox.clone();
        let device_list_clone = device_list.clone();

        let device_combobox_controller = Arc::new(tokio::sync::Mutex::new(Controller {
            channel: mpsc::channel(10),
            update: Box::new(move || {
                device_combobox.clear(&ui2);
                for device in &(*device_list_clone.lock().unwrap()) {
                    device_combobox.append(&ui2, device.name.as_str());
                }
            }),
        }));

        let view = device_combobox_controller.clone();
        ui.spawn(async move { view.lock().await.run().await });

        world.create_entity().with(Update(device_combobox_controller.clone().lock().await.channel.0.clone())).build();
    }

    let mut update_dispatcher = DispatcherBuilder::new().with(UpdateSys{handle: Handle::current()}, "update_sys", &[]).build();

    {
        let device_list = device_list.clone();

        refresh_button.on_clicked(&ui, move |_| {
            *device_list.lock().unwrap() = vec![
                Device { name: String::from("Device 1") },
                Device { name: String::from("Device 2") },
                Device { name: String::from("Device 3") },
            ];
            update_dispatcher.dispatch(&mut world);
        });
    }

    win.set_child(&ui, vert_box);

    win.show(&ui);
    // Run the application
    let mut i = 0;
    let mut ev = ui.event_loop();
    ev.on_tick(&ui, || {
        println!("tick {i}");
        i += 1;
    });
    async {
        ev.run(&ui);
    }.await;

    Ok(())
}
