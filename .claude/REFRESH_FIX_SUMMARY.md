# USB Mux Refresh Fix

## Problem
When clicking "Refresh" in vmgui, the second refresh attempt failed with:
```
Failed to connect to mux: failed to find mux interface with required endpoints
```

## Root Cause
The original implementation would:
1. Connect to the mux device and claim its USB interface
2. Leak the MuxDevice using `std::mem::forget()` to keep it alive
3. On subsequent refreshes, attempt to claim the same interface again
4. Fail because the interface was still claimed from the first connection

Windows USB doesn't allow claiming an already-claimed interface, even within the same process.

## Solution
Implemented a global cache for MuxDevice connections that:
1. Stores active mux connections in a static HashMap keyed by device info
2. Reuses existing connections on subsequent refreshes instead of creating new ones
3. Only creates new connections when a mux is encountered for the first time

## Changes Made

### 1. Added Global Cache (config_window.rs:25-31)
```rust
// Global cache for mux devices to prevent re-claiming interfaces
fn get_mux_cache() -> &'static Mutex<HashMap<String, MuxDevice>> {
    use std::sync::OnceLock;
    static MUX_CACHE: OnceLock<Mutex<HashMap<String, MuxDevice>>> = OnceLock::new();
    MUX_CACHE.get_or_init(|| Mutex::new(HashMap::new()))
}
```

### 2. Modified refresh_device_list Logic (config_window.rs:274-320)
- Check cache first before attempting to connect
- Only call `MuxDevice::connect_usb()` if device not cached
- Store new connections in cache for future use
- Removed the `std::mem::forget(active_hubs)` pattern

## Benefits
- Multiple refreshes work correctly
- No interface claiming conflicts
- Existing connections are reused (more efficient)
- Mux devices remain alive throughout application lifetime

## Testing
Run vmgui and test multiple refresh cycles:
```bash
cd D:\ody\ats-vision-tool
cargo run --release --bin vmgui
```

Click the "Refresh" button multiple times. You should see:
- First refresh: "Attempting to connect to mux..." followed by "caching for future use"
- Subsequent refreshes: "Reusing cached mux connection"
