#!/bin/bash
# Expose the Pi's USB-C port as a CDC-ACM serial device (/dev/ttyGS0) with the
# DJI VCOM VID:PID expected by the E-Port V2 (aircraft side is the USB host).
set -e

G=/sys/kernel/config/usb_gadget/dji_psdk

modprobe libcomposite
mountpoint -q /sys/kernel/config || mount -t configfs none /sys/kernel/config

# Tear down a previous instance so the script can be re-run.
if [ -d "$G" ]; then
    echo "" > "$G/UDC" 2>/dev/null || true
    rm -f "$G"/configs/c.1/acm.usb0
    rmdir "$G"/configs/c.1/strings/0x409 "$G"/configs/c.1 \
          "$G"/functions/acm.usb0 "$G"/strings/0x409 "$G" 2>/dev/null || true
fi

mkdir -p "$G"
cd "$G"
echo 0x2CA3 > idVendor
echo 0xF002 > idProduct
echo 0x0100 > bcdDevice
echo 0x0200 > bcdUSB

mkdir -p strings/0x409
echo "PSDK0001"     > strings/0x409/serialnumber
echo "Raspberry Pi" > strings/0x409/manufacturer
echo "PSDK VCOM"    > strings/0x409/product

mkdir -p configs/c.1/strings/0x409
echo "CDC ACM" > configs/c.1/strings/0x409/configuration
echo 250 > configs/c.1/MaxPower

mkdir -p functions/acm.usb0
ln -s functions/acm.usb0 configs/c.1/

ls /sys/class/udc | head -n1 > UDC
echo "USB gadget bound to $(cat UDC): /dev/ttyGS0"
