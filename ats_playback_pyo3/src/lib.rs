use ats_usb::packet::GeneralConfig;
use ats_usb::packet::Packet;
use pyo3::prelude::*;
use pyo3::wrap_pyfunction;

use std::path::PathBuf;

#[pyfunction]
pub fn read_file(path: &str) -> PyResult<(GeneralConfig, Vec<(i128, Packet)>)> {
    match ::ats_playback::read_file(&PathBuf::from(path)) {
        Ok((general_config, packets)) => Ok((general_config, packets)),
        Err(e) => Err(PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("{:?}", e))),
    }
}

#[pymodule]
fn ats_playback(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(read_file, m)?)?;
    Ok(())
}
