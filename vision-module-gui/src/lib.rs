pub mod packet;
pub mod device;
pub mod config_window;
pub mod layout_macro;
pub mod mot_runner;
pub mod test_procedure;

pub trait CloneButShorter: Clone {
    /// Use mainly for GUI code.
    fn c(&self) -> Self {
        self.clone()
    }
}

impl<T: Clone> CloneButShorter for T {}
