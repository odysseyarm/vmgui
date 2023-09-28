use iui::controls::{Button, GridAlignment, GridExpand, Group, Label, LayoutGrid, VerticalBox, Entry, HorizontalBox};
use iui::prelude::*;

fn main() {
    // Initialize the UI library
    let ui = UI::init().expect("Couldn't initialize UI library");
    // Create a window into which controls can be placed
    let mut win = Window::new(&ui, "Test App", 200, 200, WindowType::NoMenubar);

    let mut grid = LayoutGrid::new(&ui);
    grid.set_padded(&ui, true);

    let bank_label = Label::new(&ui, "Bank");
    grid.append(
        &ui,
        bank_label.clone(),
        0,
        0,
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
        1,
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
        2,
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
        0,
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
        1,
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
        2,
        1,
        1,
        GridExpand::Horizontal,
        GridAlignment::Fill,
        GridAlignment::Fill,
    );

    let mut buttons_hbox = HorizontalBox::new(&ui);
    buttons_hbox.set_padded(&ui, true);
    grid.append(&ui, buttons_hbox.clone(), 0, 3, 2, 1,
        GridExpand::Horizontal,
        GridAlignment::Fill,
        GridAlignment::Fill,
    );
    let read_button = Button::new(&ui, "Read");
    buttons_hbox.append(&ui, read_button.clone(), LayoutStrategy::Stretchy);
    let write_button = Button::new(&ui, "Write");
    buttons_hbox.append(&ui, write_button.clone(), LayoutStrategy::Stretchy);

    // let mut loading = Label::new(&ui, "Loading");

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
