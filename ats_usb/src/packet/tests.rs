use super::{CombinedMarkersReport, ObjectReport};

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
