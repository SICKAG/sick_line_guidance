#!/bin/bash

# install packages required for peak can device driver

if [ -f ../../../../../../peak_systems/pcanview-ncurses_0.8.7-0_amd64.deb ] ; then
  pushd ../../../../../../peak_systems
  sudo apt-get install libncurses5
  sudo dpkg --install pcanview-ncurses_0.8.7-0_amd64.deb
  popd
fi

# build and install peak can device driver

if [ -d ../../../../../../peak_systems/peak-linux-driver-8.7.0 ] ; then
  pushd ../../../../../../peak_systems/peak-linux-driver-8.7.0
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
  # ./driver/lspcan --all     # should print "pcanusb32" and pcan version
  # tree /dev/pcan-usb        # should show a pcan-usb device
  # ip -a link                # should print some "can0: ..." messages
  ip -details link show can0  # should print some details about "can0" net device
  popd
fi

