#!/bin/bash
if [ $# -eq 0 ]; then
    echo "Usage: config_network_lcm.sh -I [interface]"
    echo "or config_network_lcm.sh [computer]"
    echo "interface: network interface to configure"
    echo "computer: use stored interface for computer"
    echo "current computers:"
    echo " name       interface    description"
    echo " ----       --------     -----------"
    echo " thinkpad   enx70886b8ce736	left usb port close to HDMI"   
fi

if [ "$1" == "-I" ]; then
    sudo ifconfig $2 multicast
    sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev $2
fi

if [ "$1" == "thinkpad" ]; then
    sudo ifconfig enx70886b8ce736 multicast
    sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enx70886b8ce736
fi
