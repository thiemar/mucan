#!/bin/sh
# Update µCAN firmware over USB DFU.
#
# Usage: flashdfu.sh [serial-device] [firmware.bin]
#
# With a serial device, the running adapter is first asked to restart into
# its DFU bootloader. Without one, the adapter must already be in DFU mode.

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd) || exit 1

DEVICE="$1"
FIRMWARE="${2:-$SCRIPT_DIR/../firmware/app.bin}"
DFU_ID="0483:df11"

if [ ! -f "$FIRMWARE" ]; then
    echo "$FIRMWARE not found: run make first" >&2
    exit 1
fi

if [ -n "$DEVICE" ]; then
    python3 "$SCRIPT_DIR/startdfu.py" "$DEVICE" || exit 1
fi

# Wait up to 10 s for the ROM bootloader to enumerate
tries=0
until dfu-util -l -d "$DFU_ID" 2>/dev/null | grep -q "Found DFU"; do
    tries=$((tries + 1))
    if [ "$tries" -ge 50 ]; then
        echo "No DFU device $DFU_ID found" >&2
        exit 1
    fi
    sleep 0.2
done

# :leave starts the new firmware straight away. dfu-util can report an error
# as the device leaves DFU mode, so judge success by its download message.
output=$(dfu-util -d "$DFU_ID" -a 0 -s 0x08000000:leave -D "$FIRMWARE" 2>&1)
status=$?
printf '%s\n' "$output"

if printf '%s' "$output" | grep -q "File downloaded successfully"; then
    echo "Firmware updated"
    exit 0
fi
exit "$status"
