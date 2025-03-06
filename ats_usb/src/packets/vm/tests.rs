use crate::packets::vm::{Parse as _, Serialize as _};

use super::{CombinedMarkersReport, GeneralConfig, ObjectReport};

#[test]
fn test_combined_marker_report_parse_serialize() {
    let report = CombinedMarkersReport::default();
    let mut serialized = vec![];
    report.serialize(&mut serialized);
    assert_eq!(serialized.len(), CombinedMarkersReport::SIZE as usize);
    let deserialized_report = CombinedMarkersReport::parse(&mut &serialized[..]).unwrap();
    assert_eq!(deserialized_report, report);
}

#[test]
fn test_object_report_parse_serialize() {
    let report = ObjectReport::default();
    let mut serialized = vec![];
    report.serialize(&mut serialized);
    assert_eq!(serialized.len(), ObjectReport::SIZE as usize);
    let deserialized_report = ObjectReport::parse(&mut &serialized[..]).unwrap();
    assert_eq!(deserialized_report, report);
}

#[test]
fn test_general_config_parse_serialize() {
    let general_config = GeneralConfig::default();
    let mut serialized = vec![];
    general_config.serialize_to_vec(&mut serialized);
    assert_eq!(serialized.len(), GeneralConfig::SIZE as usize);
    let deserialized_general_config =
        GeneralConfig::parse(&mut &serialized[..]).unwrap();
    assert_eq!(deserialized_general_config, general_config);
}
