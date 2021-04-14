#!/bin/bash
#this script configures the p5p1 interface on the robot with a static IP.
echo -e "\033[0;33m Configuring network for static ip address 10.0.0.22...\033[0m \n"
#network interface down
sudo ip link set dev p5p1 down
#remove old IP addresses
sudo ip addr flush dev p5p1
#add address
sudo ip addr add 10.0.0.22/24 dev p5p1
#enable adapter
sudo ip link set dev p5p1 up
#show status
ip addr show dev p5p1
echo -e "\033[0;33m Setting up multicast for LCM...\033[0m \n"
#enable multicast support
sudo ifconfig p5p1 multicast
#add multicast route
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev p5p1
echo -e "\033[0;32m Done! IP address is now: \033[0m \n"
#display device IP's
hostname -I
