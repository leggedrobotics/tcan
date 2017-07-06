# Threaded Communication and Networking library #

## Overview

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=tcan)](http://rsl-ci.ethz.ch/job/tcan/)

The source code is released under a [proprietary license](LICENSE).

**Author(s):** Philipp Leemann, Christian Gehring, Remo Diethelm

## Dependencies

Common

- [message_logger](https://bitbucket.org/leggedrobotics/message_logger)

Ethercat

- [openethercat_soem](https://bitbucket.org/leggedrobotics/openethercat_soem)


## Usage

See tcan_example for an easy example how to use tcan library. tcan has two modes: synchronous and asynchronous:

- In asynchronous mode, the library creates three threads for each bus: a thread that handles incoming CAN messages, one that sends outgoing CAN messages and one that checks if devices/SDOs have timed out (sanityCheck).
- In synchronous mode, it is up to the user to call the BusManagers readMessagesSynchronous(), writeMessagesSynchronous() and sanityCheckSynchronous() functions in his main loop.


To prevent overflow of the output buffer of the SocketCAN driver (which is used by the SocketBus class) there are two possible approaches:

- Set the output queue length to a value which is large enough to hold the messages of one cycle:
    ```sudo ip link set can0 txqueuelen 100```
- Setting the SocketBusOptions::sndBufLength_ to 1 (or any other small value > 0). This sets the socket buffer size to its minimal value and will make the socket blocking if this buffer is full (which is NOT the same as the buffer of the underlying netdevice)

## Setting up the interface

### Virtual can interface

Use the ```vcan.sh``` script provided in ```tcan/scripts```:

```
#!bash

rosrun tcan vcan.sh {start|stop|restart} <name>
```
e.g.


```
#!bash

rosrun tcan vcan.sh start can0
```

### CAN-USB Adapter

Use the ```canusb.sh``` script provided in ```tcan/scripts```:

```
#!bash

rosrun tcan canusb.sh {start|stop|restart} [<dev>] <name> [<can_rate>]
```
e.g.:
```
#!bash

rosrun tcan canusb.sh start /dev/ttyUSB0 can0 -s8

rosrun tcan canusb.sh stop can0
```
where ```-s8``` sets the can baud rate according to the following table:


| flag | bitrate |
|---|---|
| -s0 | 10kbit |
| -s1 | 20Kbit |
| -s2 | 50Kbit |
| -s3 | 100Kbit |
| -s4 | 125Kbit |
| -s5 | 250Kbit |
| -s6 | 500Kbit |
| -s7 | 800Kbit |
| -s8 | 1Mbit |



### Peak PCIe card

The linux kernel >= 2.6.25 supports Socketcan natively. To setup the interface, you can use


```
#!bash

sudo ip link set can0 type can bitrate 1000000 berr-reporting on
sudo ip link set can0 up
```

However the performance of the standard linux kernel driver may be bad (large gaps between can frames). Peak provides their own Socketcan driver, which can be downloaded from http://www.peak-system.com/fileadmin/media/linux/index.htm. Unpack the archive and install it with


```
#!bash

make clean
make -C driver NET=NETDEV_SUPPORT
sudo make install
```

The driver can be uninstalled with

```
#!bash

sudo make uninstall
```

To set the baudrate, the above ```ip``` command cannot be used, do the following instead:

```
#!bash

echo "i 0x001C" > /dev/pcan0
```

where ```0x001C``` is a hex representation of the baudrate, which can be taken from the following table:

| hex | bitrate |
|---|---|
| 0x7F7F | 5kbit |
| 0x672F | 10kbit |
| 0x532F | 20Kbit |
| 0x472F | 50Kbit |
| 0x432F | 100Kbit |
| 0x031C | 125Kbit |
| 0x011C | 250Kbit |
| 0x001C | 500Kbit |
| 0x0014 | 1Mbit |