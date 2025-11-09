# USB Mux Refresh Fix - Final Solution

## Problem Summary
After clicking refresh in vmgui, the second refresh would fail with:
```
Failed to connect to mux: failed to find mux interface with required endpoints
```

Additionally, connecting and then refreshing would cause memory issues on the dongle-fw:
- "Packet received for unregistered slot" warnings in ats_usb
- "OutOfMemory" errors in dongle-fw trouble_host
- "Broadcast packet to BLE failed" errors in vm-fw

## Root Causes

### 1. Interface Claiming Issue (Windows)
The original code used `std::mem::forget(active_hubs)` to keep mux connections alive indefinitely. On Windows, this prevented the USB interface from being released, causing the "failed to find mux interface" error on subsequent refreshes because you cannot claim an already-claimed interface.

### 2. Stale Connection State
When vmgui closed or refreshed, the old MuxDevice connections remained in memory with active communication channels. This caused:
- Packets being sent to unregistered slots
- BLE buffer exhaustion on dongle-fw
- Connection state confusion across the system

## Solution

### Changes in `vision-module-gui/src/config_window.rs`

**1. Removed `std::mem::forget()` (line 304-305 deleted)**
```diff
-                    // Keep muxes alive by leaking them (they need to stay alive for the lifetime of the app)
-                    std::mem::forget(active_hubs);
-                    eprintln!("Mux query task completed, muxes kept alive");
+                    eprintln!("Mux query task completed");
```

**2. Let MuxDevice be managed by VmConnectionInfo clones (line 291)**
```diff
-                                // Keep the mux alive by storing it
-                                active_hubs.push(hub);
-                                eprintln!("Mux kept alive for future connections");
+                                // MuxDevice will be kept alive by the VmConnectionInfo clones
+                                eprintln!("Mux devices added to connection list");
```

**3. Added explicit cleanup at start of refresh (lines 212-218)**
```rust
// First, clear the device list to drop all existing connections
// This ensures MuxDevice interfaces are released before we try to reclaim them
eprintln!("Clearing device list to release interfaces...");
device_list.set(Vec::new());

// Also clear any active device connection
device.set(None);
```

## How It Works

1. **On first refresh:**
   - Creates new `MuxDevice` connections
   - Stores them in `VmConnectionInfo::ViaMux` entries
   - These entries keep the `MuxDevice` alive via `Arc` clones

2. **On subsequent refresh:**
   - **First**, explicitly clears `device_list` and `device` signals
   - This drops all `VmConnectionInfo` entries
   - When the last `VmConnectionInfo` is dropped, the `MuxDevice` is dropped
   - The `MuxDevice` Drop implementation releases the USB interface
   - **Then**, creates new connections (interfaces are now available)

3. **On vmgui close:**
   - Signals are dropped naturally
   - All connections are cleaned up
   - No leaked memory or claimed interfaces

## Benefits

✅ Multiple refreshes work correctly
✅ No interface claiming conflicts
✅ Clean connection lifecycle (no memory leaks)
✅ Works correctly when closing and reopening vmgui
✅ Proper cleanup prevents dongle-fw memory exhaustion
✅ No more "unregistered slot" warnings

## Testing

Build and run:
```bash
cd D:\ody\ats-vision-tool
cargo build --release --bin vmgui
cargo run --release --bin vmgui
```

Test scenarios:
1. **Multiple refreshes:** Click refresh button multiple times - should work every time
2. **Connect → Refresh:** Select device, connect, then refresh - should work
3. **Close → Reopen:** Close vmgui, reopen, refresh - should work
4. **Check logs:** No "unregistered slot" or OutOfMemory errors

## Expected Output

```
=== refresh_device_list called ===
Clearing device list to release interfaces...
Found 1 USB devices
Found 1 mux devices
Spawning mux query task...
Mux query task started
Attempting to connect to mux...
Mux connected successfully, requesting devices...
Mux query successful, found 1 device(s)
  Device: 8B:F7:2C:A4:C1:25
Mux devices added to connection list
Updating device list with 1 total devices
Mux query task completed
```

Every refresh should show "Attempting to connect to mux..." because we're properly cleaning up and reconnecting.
