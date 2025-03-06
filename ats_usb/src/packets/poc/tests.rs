use super::{GeneralConfig, PacketType};

#[test]
fn test_general_config_parse_serialize() {
    let general_config = GeneralConfig::default();
    let mut serialized = vec![];
    general_config.serialize(&mut serialized);
    assert_eq!(serialized.len(), GeneralConfig::SIZE as usize);
    let deserialized_general_config =
        GeneralConfig::parse(&mut &serialized[..], PacketType::ReadConfigResponse()).unwrap();
    assert_eq!(deserialized_general_config, general_config);
}
