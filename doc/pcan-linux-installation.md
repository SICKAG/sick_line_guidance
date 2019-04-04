# pcan-linux-installation

ROS support for sick line guidance sensors has been developed and tested using PEAK CAN adapter for the CAN communication. 
This is a quick howto install linux driver for the pcan usb adapter.

# Installation

The following installation is recommended:

&nbsp; 1. Download PCAN-View for Linux from https://www.peak-system.com/linux/ or https://www.peak-system.com/quick/ViewLinux_amd64/pcanview-ncurses_0.8.7-0_amd64.deb and install:
```bash
sudo apt-get install libncurses5
sudo dpkg --install pcanview-ncurses_0.8.7-0_amd64.deb
```

&nbsp; 2. If you're running ROS in a virtual machine, make sure the usb-port for the PCAN-USB-adapter is connected to your VM. 

&nbsp; 3. Download and unzip peak-linux-driver-8.7.0.tar.gz from https://www.peak-system.com/linux/ or https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-8.7.0.tar.gz

&nbsp; 4. Install the linux driver and required packages:
```bash
cd peak-linux-driver-8.7.0
# install required packages
sudo apt-get install can-utils
sudo apt-get install gcc-multilib
sudo apt-get install libelf-dev
sudo apt-get install libpopt-dev
sudo apt-get install tree
# build and install pcan driver
make clean
make NET=NETDEV_SUPPORT
sudo make install
# install the modules
sudo modprobe pcan
sudo modprobe can
sudo modprobe vcan
sudo modprobe slcan
# setup and configure "can0" net device
sudo ip link set can0 type can
sudo ip link set can0 up type can bitrate 125000 # configure the CAN bitrate, f.e. 125000 bit/s
# check installation
./driver/lspcan --all       # should print "pcanusb32" and pcan version
tree /dev/pcan-usb          # should show a pcan-usb device
ip -a link                  # should print some "can0: ..." messages
ip -details link show can0  # should print some details about "can0" net device
```
Example output after successfull installation of a pcan usb adapter:
```bash
user@ubuntu-ros:~/peak-linux-driver-8.7.0$ ./driver/lspcan --all
pcan version: 8.7.0
pcanusb32	CAN1CAN1	8MHz	500k	CLOSED	-	0	0	0
user@ubuntu-ros:~/peak-linux-driver-8.7.0$ tree /dev/pcan-usb
/dev/pcan-usb
├── 0
│   └── can0 -> ../../pcanusb32
└── devid=5 -> ../pcanusb32
user@ubuntu-ros:~/peak-linux-driver-8.7.0$ ip -a link
3: can0: <NOARP> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
    link/can 
user@ubuntu-ros:~/peak-linux-driver-8.7.0$ ip -details link show can0
3: can0: <NOARP> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
    link/can  promiscuity 0 
    can state STOPPED restart-ms 0 
	  bitrate 500000 sample-point 0.875 
	  tq 125 prop-seg 6 phase-seg1 7 phase-seg2 2 sjw 1
	  pcan: tseg1 1..16 tseg2 1..8 sjw 1..4 brp 1..64 brp-inc 1
	  clock 8000000numtxqueues 1 numrxqueues 1 gso_max_size 65536 gso_max_segs 65535 
```

See https://www.peak-system.com/fileadmin/media/linux/files/PCAN-Driver-Linux_UserMan_eng.pdf for further details.
