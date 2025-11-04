# Changes needed for config_window.rs

## Summary
This implementation queries the hub for actual device addresses during enumeration,
so all devices (both direct USB and hub-connected) will have real addresses in the dropdown.

## Line-by-line changes:

### 1. Line 9 - Update imports
BEFORE: `device::VmDevice,`
AFTER: `device::{VmDevice, VmConnectionInfo, HubDevice},`

### 2. Line 115 - Change device list type
BEFORE: `let device_list = create_rw_signal(Vec::<DeviceInfo>::new());`
AFTER: `let device_list = create_rw_signal(Vec::<VmConnectionInfo>::new());`

### 3. Line 135 - Update connection call
BEFORE: `Ok(VmDevice::connect_usb(_device).await.unwrap())`
AFTER: `Ok(_device.connect().await.unwrap())`

### 4. Line 190 - Update display function call
BEFORE: `display_for_device_info(device)`
AFTER: `display_for_vm_connection(device)`

### 5. Lines 202-238 - Replace refresh_device_list function
Replace with the implementation from /tmp/refresh_device_list_with_query.rs
This queries the hub asynchronously for connected devices and adds them with real addresses.

### 6. Lines 1034-1055 - Replace display function
Replace with the implementation from /tmp/display_function.rs
This handles both DirectUsb and ViaHub variants.

## Key features:
- Queries hub during enumeration for actual device addresses
- Shows direct USB devices immediately
- Asynchronously queries hub and updates list when hub devices are found
- All devices shown with real addresses (no placeholders)
- User sees "VM via Hub AABBCCDDEEFF" for hub-connected devices
