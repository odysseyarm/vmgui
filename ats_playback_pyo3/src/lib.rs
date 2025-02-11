use ats_usb::packets::vm::GeneralConfig;
use ats_usb::packets::vm::Packet;
use pyo3::prelude::*;
use pyo3::wrap_pyfunction;

use std::path::PathBuf;

#[pyfunction]
pub fn read_file(path: &str) -> PyResult<(GeneralConfig, Vec<(u128, Packet)>)> {
    match ::ats_playback::read_file(&PathBuf::from(path)) {
        Ok((general_config, packets)) => Ok((general_config, packets)),
        Err(e) => Err(PyErr::new::<pyo3::exceptions::PyIOError, _>(format!(
            "{:?}",
            e
        ))),
    }
}

#[pyfunction]
pub fn peak_detect_128(mut a: [f32; 128]) -> f32 {
    ats_cv::peak_detect_128(&mut a, 45.0, 85.0, 200.0)
}

#[pymodule]
fn ats_playback(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(read_file, m)?)?;
    m.add_function(wrap_pyfunction!(peak_detect_128, m)?)?;
    Ok(())
}
