# Hub Support Implementation - Status

## Current Issue

You're seeing "failed to open device (error 5)" because:

1. **Multiple refresh attempts**: Each time you click "Refresh", it spawns a new async task to query the hub
2. **Windows USB exclusivity**: Windows only allows one process/handle to open a USB device at a time
3. **Race condition**: If you click refresh while a previous hub query is still running, the second attempt fails with error 5 (Access Denied)

## Solutions

### Option 1: Prevent concurrent queries (Recommended)
Add a mutex/flag to prevent multiple simultaneous hub queries. Only allow one refresh at a time.

### Option 2: Use a single persistent hub connection
Keep the hub connection open and reuse it, but this complicates lifecycle management.

### Option 3: Just handle the error gracefully
The current implementation already handles the error and continues - you'll just see the error message in the console.

## Current Behavior

The implementation DOES work correctly if you:
1. Click "Refresh" once
2. Wait for the hub query to complete (you'll see "Hub query successful" in console)
3. The hub-connected devices will appear in the dropdown

The "failed to connect" errors you see are from clicking refresh multiple times quickly.

## Recommendation

Since the error is benign (it just means a concurrent query failed, but one will succeed), and clicking refresh once works fine, I recommend:

**Option A**: Add user feedback - show "Querying hub..." status while query is in progress
**Option B**: Disable refresh button while query is running
**Option C**: Accept current behavior - it's harmless, just noisy in logs

Let me know which approach you prefer!
