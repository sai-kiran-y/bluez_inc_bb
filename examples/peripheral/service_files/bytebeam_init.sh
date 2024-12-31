#!/bin/bash
/home/root/4g_on.sh
/home/root/CAN_init.sh
/home/root/ble_init.sh
/etc/ppp/peers/ppp_on.sh
/home/root/ble_app
/home/root/uplink -c /home/root/config.toml -a /home/root/device.json -vvv
/home/root/cancollector-hf -c can0 -s can_raw -a localhost:5050
