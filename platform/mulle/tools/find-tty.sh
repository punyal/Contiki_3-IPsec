#!/bin/bash

# Usage:
# find-tty.sh [serial] - gives first tty assigned to the given serial number.

# Find all FT2232H devices
for dev in /sys/bus/usb/devices/*; do
    if [ ! -f "${dev}/idVendor" ]; then
        # not a main device
        continue
    fi
    # filter out any devices not identified as 0403:6010
    if grep -v '0403' "${dev}/idVendor" -q; then
        continue
    fi
    if grep -v '6010' "${dev}/idProduct" -q; then
        continue
    fi
    serial=$(cat "${dev}/serial")
    # Look if any subdevices have a tty directory, this means that it is assigned a port.
    unset ttys
    ttys=$(ls "${dev}:"*/tty* -d -1 | xargs -n 1 basename)
    if [ -z "${ttys}" ]; then
        continue
    fi
    # split results into array
    read -r -a ttys <<< ${ttys}
    for s in "${@}"; do
        if [ "${serial}" == "${s}" ]; then
            # return first tty
            echo "/dev/${ttys[0]}"
            exit 0;
        fi
    done
done
# not found
exit 1;
