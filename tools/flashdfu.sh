#!/bin/sh
# Keep the original entry point and resolve its helper without changing cwd.
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd) || exit 1
exec python3 "$SCRIPT_DIR/flashdfu.py" "$@"
