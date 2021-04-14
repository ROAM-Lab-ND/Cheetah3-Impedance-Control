## General
The 3DoFTest-ADL folder contains the code to run on cheetah computer.

The 3DoFTest-GUI folder contains the code to run on the development computer.

Build instructions could be found in each corresponding folder.

Follow the instructions below to set up the network configurations.

## Network Config
### SSH to Cheetah3
### SSH to Cheetah3 via Internet

On robot, configure the interface p5p1 to connect to internet using:

    sudo ifconfig p5p1 up
    sudo dhclient

Ping to google to check the success of connection using

    ping www.google.com

On development computer, SSH to robot using

    ssh robot@nd-cheetah

Copy the folder Impedance-Control-ADL to the robot computer (ADL) under home folder.

The steps above are not necessary if you already have Impedance-Control-ADL on the cheetah computer and would like to connect to the cheetah computer from the development computer via wired Ethernet.

### SSH to Cheetah3 using local network
On robot computer (ADL), configure the interface p5p1 for static IP 10.0.0.22 using

    cd ~/3DoFTest-ADL/scripts
    ./config_network_ADL.sh

Note that there are two ethernet interfaces and one ethercat interface on the ADL board (ethercat share the port with one of the ethernet interface). The ethercat interface is em1, while the other ethernet interface is p5p1. Since we use em1 for ethercat, p5p1 should be configured for LCM. **Make sure you are using p5p1 rather than em1 in config_network_ADL.sh**. The `./config_network_ADL.sh` would also configure p5p1 for LCM.

Configure the interface on laptop to use static IP 10.0.0.12 (other address should also work). The interface should be the one connected to p5p1 on ADL. Make sure the netmask on both are the same.

SSH to robot using

    ssh robot@10.0.0.22

## LCM Config

Configure the interface on laptop for LCM using

     cd Impedance-Control-GUI/scripts
    ./config_network_lcm.sh

Set the environment variable LCM_DEFAULT_URL using

    export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1

Setting the ttl to 1 allows the packets to be published on the local network (accross computers). Setting ttl to 0 only allows communication on local [computer](https://lcm-proj.github.io/group__LcmC__lcm__t.html#gaf29963ef43edadf45296d5ad82c18d4b).

## LCM Test
On development computer, 
        
        cd ~/lcm/examples
        gcc -o listener listener.c exlcm_example_t.c -llcm
        ./listener

On robot computer,

        cd ~/lcm/examples
        gcc -o sender send_message.c exlcm_example_t.c -llcm
        ./sender

If succeed, you would see printint out on laptop.

## Impedance Control Test
On robot computer,
    
        cd Impedance-Control-ADL/build
        sudo -E test/linux/simple_test/simple_test em1

On development computer,

        cd Impedance-Control-GUI/build
        ./Legcontrol    


