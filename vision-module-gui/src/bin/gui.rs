use std::fs::File;
use std::sync::Arc;

use anyhow::Result;
use bevy::app::{App, Startup, Update};
use bevy::ecs::query::With;
use bevy::ecs::schedule::IntoSystemConfigs;
use bevy::ecs::system::{Commands, Query};
use bevy::window::{WindowPlugin, WindowCloseRequested};
use bevy::winit::WinitPlugin;
use bevy::DefaultPlugins;
use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    prelude::*,
    window::{CursorGrabMode, PresentMode, WindowLevel, WindowTheme},
};
use bevy_infinite_grid::{
    GridShadowCamera, InfiniteGridBundle, InfiniteGridPlugin, InfiniteGridSettings,
};
use iui::prelude::*;
use leptos_reactive::{create_effect, RwSignal, SignalGet, SignalGetUntracked, SignalSet, SignalWith};
use nalgebra::{Const, Vector2};
use tracing::Level;
use tracing_subscriber::EnvFilter;
use vision_module_gui::packet::GeneralConfig;
use vision_module_gui::{config_window, marker_config_window, Frame};
use vision_module_gui::{CloneButShorter, MotState};
use tokio::sync::Mutex;
use tokio::task::AbortHandle;
use iui::controls::{Area, HorizontalBox, FileTypeFilter};
use vision_module_gui::mot_runner::MotRunner;
use vision_module_gui::run_canvas::RunCanvas;
use vision_module_gui::test_canvas::TestCanvas;

// Things to avoid doing
// * Accessing signals outside of the main thread
//     * Getting panics, setting fails silently
//     * by extension, don't access signals from tokio::spawn, use ui.spawn
// * Excessive blocking in the main thread or in callbacks
//     * Freezes the GUI
// * Creating libui uiControls dynamically
//     * They are leaky
// * Holding mutexes and setting signals
//     * Drop the mutex, or use ui_ctx.queue_main()

#[derive(bevy::ecs::component::Component)]
struct Name(String);

#[derive(bevy::ecs::component::Component)]
struct Person;

#[derive(Resource)]
struct MotRunnerResource(Arc<Mutex<MotRunner>>);

enum Event {
    OpenWindow,
}

#[derive(Resource, Deref)]
struct StreamReceiver(crossbeam::channel::Receiver<Event>);

#[derive(Event)]
struct StreamEvent(Event);

fn read_stream(receiver: Res<StreamReceiver>, mut events: EventWriter<StreamEvent>) {
    for from_stream in receiver.try_iter() {
        events.send(StreamEvent(from_stream));
    }
}

fn show_window(mut commands: Commands, mut reader: EventReader<StreamEvent>, mut window: Query<&mut bevy::window::Window>) {
    for (per_frame, event) in reader.read().enumerate() {
        match event.0 {
            Event::OpenWindow => {
                if window.iter().len() == 0 {
                    commands.spawn(create_bevy_window(true));
                } else {
                    window.single_mut().visible = true;
                }
            },
        }
    }
}

pub fn hide_instead_of_close(mut closed: EventReader<WindowCloseRequested>, mut window: Query<&mut bevy::window::Window>) {
    for event in closed.read() {
        if let Ok(mut window) = window.get_mut(event.window) {
            window.visible = false;
        }
    }
}

fn hello_world() {
    println!("hello world!");
}

fn update_people(mut query: Query<&mut Name, With<Person>>) {
    for mut name in &mut query {
        if name.0 == "Elaina Proctor" {
            name.0 = "Elaina Hume".to_string();
            break; // We don’t need to change any other names
        }
    }
}

fn greet_people(query: Query<&Name, With<Person>>) {
    for name in &query {
        println!("hello {}!", name.0);
    }
}

fn create_bevy_window(visible: bool) -> bevy::window::Window {
    bevy::window::Window {
        title: "I am a window!".into(),
        name: Some("bevy.app".into()),
        resolution: (500., 300.).into(),
        present_mode: PresentMode::AutoVsync,
        // Tells wasm not to override default event handling, like F5, Ctrl+R etc.
        prevent_default_event_handling: false,
        window_theme: Some(WindowTheme::Dark),
        enabled_buttons: bevy::window::EnabledButtons {
            maximize: false,
            ..Default::default()
        },
        // This will spawn an invisible window
        // This is useful when you want to avoid the white window that shows up before the GPU is ready to render the app.
        visible,
        ..default()
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Spawn a cube to rotate.
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::default()),
            material: materials.add(Color::WHITE),
            transform: Transform::from_translation(Vec3::ZERO),
            ..default()
        },
    ));

    commands.spawn(InfiniteGridBundle {
        settings: InfiniteGridSettings {
            // shadow_color: None,
            ..default()
        },
        ..default()
    });

    // Spawn a camera looking at the entities to show what's happening in this example.
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 10.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        GridShadowCamera,
    ));

    // Add a light source so we can see clearly.
    commands.spawn(DirectionalLightBundle {
        transform: Transform::from_xyz(3.0, 3.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}

// This system will rotate any entity in the scene with a Rotatable component around its y-axis.
fn rotate_cube(mut camera_query: Query<(&Camera, &mut Transform)>, runner: Res<MotRunnerResource>) {
    let runner = runner.0.blocking_lock();
    let rotation = runner.state.rotation_mat.c();
    let rotation = rotation.reshape_generic(Const::<3>, Const::<3>).transpose();
    let translation = runner.state.translation_mat.c();

    let (camera, mut camera_transform) = camera_query.single_mut();
    camera_transform.translation = Vec3::new(translation.x as f32, translation.y as f32, translation.z as f32);
    let rotation = Mat3::from_cols_array(&[
        rotation[(0, 0)] as f32,
        rotation[(0, 1)] as f32,
        rotation[(0, 2)] as f32,
        rotation[(1, 0)] as f32,
        rotation[(1, 1)] as f32,
        rotation[(1, 2)] as f32,
        rotation[(2, 0)] as f32,
        rotation[(2, 1)] as f32,
        rotation[(2, 2)] as f32,
    ]);
    camera_transform.rotation = Quat::from_mat3(&rotation);
    println!("translation: {:?}", camera_transform.translation);
}

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::builder()
                .with_env_var("RUST_LOG")
                .with_default_directive(Level::INFO.into())
                .from_env_lossy(),
        )
        .init();
    let tokio_rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap();
    let tokio_handle = tokio_rt.handle();
    let _enter = tokio_handle.enter();
    let leptos_rt = leptos_reactive::create_runtime();
    // Initialize the UI library
    let ui = UI::init().expect("Couldn't initialize UI library");
    let ui_ctx = ui.async_context();

    let simulator_addr = std::env::args().nth(1);
    let datapoints: Arc<Mutex<Vec<Frame>>> = Arc::new(Mutex::new(Vec::new()));
    let state = MotState::default();
    let ui_update: RwSignal<()> = leptos_reactive::create_rw_signal(());
    let mot_runner = Arc::new(Mutex::new(MotRunner {
        state,
        device: None,
        markers_settings: Default::default(),
        record_impact: false,
        datapoints: datapoints.c(),
        ui_update: ui_update.c(),
        ui_ctx,
        nf_offset: Vector2::default(),
        general_config: GeneralConfig::default(),
    }));
    let tracking = RwSignal::new(false);
    let testing = RwSignal::new(false);
    let marker_offset_calibrating = RwSignal::new(false);

    // Create a main_window into which controls can be placed
    let mut main_win = iui::prelude::Window::new(&ui, "ATS Vision Tool", 640, 480, WindowType::NoMenubar);
    let (mut config_win, device_rs, marker_pattern_memo) = config_window::config_window(
        &ui,
        simulator_addr,
        mot_runner.c(),
        tokio_handle,
    );
    let mut marker_config_win = marker_config_window::marker_config_window(
        &ui,
        marker_offset_calibrating,
        marker_pattern_memo,
        mot_runner.c(),
    );


    let mut test_win = iui::prelude::Window::new(&ui, "Aimpoint Test", 640, 480, WindowType::NoMenubar);
    test_win.set_margined(&ui, false);
    test_win.set_borderless(&ui, true);

    let test_win_on_closing = move |_: &mut _| {
        testing.set(false);
        marker_offset_calibrating.set(false);
    };
    test_win.on_closing(&ui, test_win_on_closing.c());
    let test_area = Area::new(&ui, Box::new(TestCanvas {
        ctx: ui.c(),
        window: test_win.c(),
        on_closing: Box::new(test_win_on_closing.c()),
        runner: mot_runner.c(),
        last_draw_width: None,
        last_draw_height: None,
    }));
    let mut test_hbox = HorizontalBox::new(&ui);
    test_hbox.append(&ui, test_area.c(), LayoutStrategy::Stretchy);
    test_win.set_child(&ui, test_hbox);


    vision_module_gui::layout! { &ui,
        let vert_box = VerticalBox(padded: true) {
            Compact: let grid = LayoutGrid(padded: true) {
                (0, 0)(1, 1) Vertical (Fill, Fill) : let config_button = Button("Config")
                (1, 0)(1, 1) Vertical (Fill, Fill) : let marker_config_button = Button("Marker Config")
                (2, 0)(1, 1) Vertical (Fill, Fill) : let track_button = Button(move || {
                    if !tracking.get() { "Start Tracking" } else { "Stop Tracking" }
                })
                (3, 0)(1, 1) Vertical (Fill, Fill) : let test_button = Button("Run Test")
                (4, 0)(1, 1) Vertical (Fill, Fill) : let windowed_checkbox = Checkbox("Windowed", checked: false)
                (5, 0)(1, 1) Vertical (Fill, Fill) : let bevy_button = Button("Launch Bevy")
            }
            Compact: let separator = HorizontalSeparator()
            Compact: let spacer = Spacer()
            Compact: let form_vbox = VerticalBox(padded: true) {
                Compact: let form = Form(padded: true) {
                    (Compact, "Datapoints added:"): let collected_text = Label("")
                    (Compact, "Dataset"): let dataset_controls_group = HorizontalBox(padded: true) {
                        Compact: let add_datapoint_btn = Button("Add datapoint")
                        Compact: let remove_datapoint_btn = Button("Remove datapoint")
                        Compact: let clear_datapoints_btn = Button("Clear datapoints")
                        Compact: let record_impacts_cbx = Checkbox("Record impacts", checked: false)
                        Compact: let save_datapoints_btn = Button("Save to file")
                    }
                }
                Compact: let separator = HorizontalSeparator()
            }
            Stretchy: let run_hbox = HorizontalBox() {
                Stretchy: let run_area = Area(Box::new(RunCanvas {
                    ctx: ui.c(),
                    runner: mot_runner.c(),
                }))
            }
        }
    }
    form_vbox.hide(&ui);
    run_hbox.hide(&ui);
    main_win.set_child(&ui, vert_box.clone());

    let (tx, rx) = crossbeam::channel::bounded(10);

    std::thread::spawn({
        let mot_runner = mot_runner.c();
        move || {
            App::new().add_plugins((
                DefaultPlugins.set(WindowPlugin {
                    primary_window: Some(create_bevy_window(false)),
                    exit_condition: bevy::window::ExitCondition::DontExit,
                    close_when_requested: false,
                    ..default()
                }).set(WinitPlugin {
                    run_on_any_thread: true,
                    ..default()
                }),
                // LogDiagnosticsPlugin::default(),
                // FrameTimeDiagnosticsPlugin,
                InfiniteGridPlugin,
            ))
            .add_systems(Startup, setup)
            .add_systems(Update, (read_stream, show_window, rotate_cube, hide_instead_of_close))
            .add_event::<StreamEvent>()
            .insert_resource(StreamReceiver(rx))
            .insert_resource(MotRunnerResource(mot_runner))
            .run();
        }
    });

    bevy_button.on_clicked(&ui, move |_| {
        tx.send(Event::OpenWindow).unwrap();
    });

    windowed_checkbox.on_toggled(&ui, {
        let test_win = test_win.c();
        let ui = ui.c();
        move |checked| {
            let mut test_win = test_win.c();
            let is_visible = test_win.visible(&ui);
            if checked {
                test_win.set_fullscreen(&ui, false);
            } else {
                test_win.set_fullscreen(&ui, true);
            }
            if !is_visible {
                test_win.hide(&ui);
            }
        }
    });

    create_effect({
        let ui = ui.c();
        let collected_text = collected_text.c();
        let datapoints = datapoints.c();
        move |_| {
            ui_update.with(|_| {
                let datapoints = datapoints.c();
                let datapoints = datapoints.blocking_lock();

                let mut collected_text = collected_text.c();

                collected_text.set_text(&ui, &datapoints.len().to_string());
            });
        }
    });

    // Disable buttons if no device is connected
    create_effect({
        let ui = ui.c();
        let test_win = test_win.c();
        let track_button = track_button.c();
        let test_button = test_button.c();
        let test_win_on_closing = test_win_on_closing.c();
        move |_| {
            let mut test_win = test_win.c();
            let mut track_button = track_button.c();
            let mut test_button = test_button.c();
            device_rs.with(|device| {
                if device.is_none() {
                    test_win_on_closing.c()(&mut test_win);
                    track_button.disable(&ui);
                    test_button.disable(&ui);
                } else {
                    track_button.enable(&ui);
                    test_button.enable(&ui);
                }
            });
        }
    });

    // Start/stop the mot_runner task as needed
    create_effect({
        let mot_runner = mot_runner.c();
        move |abort_handle: Option<Option<AbortHandle>>| {
            if tracking.get() || testing.get() || marker_offset_calibrating.get() {
                if let Some(Some(abort_handle)) = abort_handle {
                    // The mot_runner task is already running
                    Some(abort_handle)
                } else {
                    Some(tokio::spawn(vision_module_gui::mot_runner::run(mot_runner.c())).abort_handle())
                }
            } else {
                if let Some(Some(abort_handle)) = abort_handle {
                    abort_handle.abort();
                }
                None
            }
        }
    });

    // Keep the mot_runner device in sync with the selected device from the config window
    create_effect({
        let view = mot_runner.c();
        move |_| {
            let view = view.c();
            view.blocking_lock().device = device_rs.get();
        }
    });

    // Show the test window and form when running test or when calibrating marker offsets
    create_effect({
        let ui = ui.c();
        let form_vbox = form_vbox.c();
        let test_win = test_win.c();
        move |_| {
            if testing.get() || marker_offset_calibrating.get() {
                form_vbox.c().show(&ui);
                test_win.c().show(&ui);
            } else {
                form_vbox.c().hide(&ui);
                test_win.c().hide(&ui);
            }
        }
    });

    // Show the run_hbox form when tracking
    create_effect({
        let ui = ui.c();
        let run_hbox = run_hbox.c();
        move |_| {
            if tracking.get() {
                run_hbox.c().show(&ui);
            } else {
                run_hbox.c().hide(&ui);
            }
        }
    });

    add_datapoint_btn.on_clicked(&ui, {
        let ui = ui.c();
        let datapoints = datapoints.c();
        let collected_text = collected_text.c();
        let state = mot_runner.c();
        move |_| {
            let datapoints = datapoints.c();
            let mut datapoints = datapoints.blocking_lock();
            let mut collected_text = collected_text.c();

            let mut frame = Frame {
                              nf_aim_point_x: None,
                              nf_aim_point_y: None,
                              };

            let runner = state.blocking_lock();
            let state = &runner.state;

            if let Some(nf_aim_point) = state.nf_aim_point {
                let nf_aim_point = nf_aim_point + runner.nf_offset;
                frame.nf_aim_point_x = Some(nf_aim_point.x);
                frame.nf_aim_point_y = Some(nf_aim_point.y);
            }

            datapoints.push(frame);
            collected_text.set_text(&ui, datapoints.len().to_string().as_str());
        }
    });

    remove_datapoint_btn.on_clicked(&ui, {
        let ui = ui.c();
        let datapoints = datapoints.c();
        let collected_text = collected_text.c();
        move |_| {
            let datapoints = datapoints.c();
            let mut datapoints = datapoints.blocking_lock();
            let mut collected_text = collected_text.c();
            datapoints.pop();
            collected_text.set_text(&ui, datapoints.len().to_string().as_str());
        }
    });

    clear_datapoints_btn.on_clicked(&ui, {
        let ui = ui.c();
        let datapoints = datapoints.c();
        move |_| {
            let datapoints = datapoints.c();
            let mut datapoints = datapoints.blocking_lock();
            datapoints.clear();
            collected_text.set_text(&ui, datapoints.len().to_string().as_str());
        }
    });

    record_impacts_cbx.on_toggled(&ui, {
        let mot_runner = mot_runner.c();
        move |checked| {
            mot_runner.blocking_lock().record_impact = checked;
        }
    });

    save_datapoints_btn.on_clicked(&ui, {
        let ui = ui.c();
        let main_win = main_win.c();
        move |_| {
            let datapoints = datapoints.c();
            let datapoints = datapoints.blocking_lock();
            let path_buf = main_win.save_file_with_filter(&ui, &[FileTypeFilter::new("csv").extension("csv")]);
            if let Some(mut path_buf) = path_buf {
                if path_buf.extension() != Some("csv".as_ref()) {
                    path_buf.as_mut_os_string().push(".csv");
                }
                let file = File::create(path_buf).expect("Could not create file");
                let mut writer = csv::Writer::from_writer(file);

                for datapoint in datapoints.as_slice() {
                    writer.serialize(datapoint).expect("Could not serialize");
                }
                writer.flush().expect("Could not flush");
            }
        }
    });

    config_button.on_clicked(&ui, {
        let ui = ui.c();
        move |_| {
            config_win.show(&ui);
        }
    });

    marker_config_button.on_clicked(&ui, {
        let ui = ui.c();
        move |_| {
            marker_config_win.show(&ui);
        }
    });

    track_button.on_clicked(&ui, move |_| tracking.set(!tracking.get_untracked()));
    test_button.on_clicked(&ui, move |_| testing.set(true));

    main_win.show(&ui);

    ui.ui_timer(5, {
        let ui = ui.c();
        let run_area = run_area.c();
        let test_area = test_area.c();
        move || {
            if tracking.get_untracked() {
                run_area.queue_redraw_all(&ui);
            }
            if testing.get_untracked() || marker_offset_calibrating.get_untracked() {
                test_area.queue_redraw_all(&ui);
            }
            true
        }
    });

    // Run the application
    let mut ev = ui.event_loop();
    ev.run(&ui);

    leptos_rt.dispose();
    Ok(())
}
