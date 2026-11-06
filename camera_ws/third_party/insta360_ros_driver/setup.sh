#!/usr/bin/env bash

set -Eeuo pipefail

RULE_FILE="/etc/udev/rules.d/99-insta.rules"
RULE='SUBSYSTEM=="usb", ATTR{idVendor}=="2e1a", GROUP="plugdev", MODE="0660", SYMLINK+="insta"'

if ! getent group plugdev >/dev/null; then
  sudo groupadd --system plugdev
fi

if ! id -nG | tr ' ' '\n' | grep -Fxq plugdev; then
  sudo usermod -aG plugdev "$(id -un)"
  echo "Added $(id -un) to plugdev. Log out and back in before using the camera."
fi

printf '%s\n' "$RULE" | sudo tee "$RULE_FILE" >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb

for _ in {1..10}; do
  if [[ -e /dev/insta ]]; then
    echo "Insta360 udev rule installed: $(readlink -f /dev/insta)"
    if [[ -r /dev/insta && -w /dev/insta ]]; then
      echo "USB device is readable and writable."
      exit 0
    fi
    echo "Rule is installed, but this login session has no access yet."
    echo "Log out/in, reconnect the camera in Android USB mode, then retry."
    exit 0
  fi
  sleep 0.5
done

echo "Rule installed at $RULE_FILE, but /dev/insta was not created." >&2
echo "Reconnect the powered-on camera in Android USB mode and run: lsusb -d 2e1a:0002" >&2
exit 1

