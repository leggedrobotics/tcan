# Threaded CAN library #

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=tcan)](http://rsl-ci.ethz.ch/job/tcan/)


## Dependencies

- [message_logger](https://bitbucket.org/ethz-asl-lr/message_logger)


## Usage ##

See tcan_example for an easy example how to use tcan library. tcan has two modes: synchronous and asynchronous:

- In asynchronous mode, the library creates three threads for each bus: a thread that handles incoming CAN messages, one that sends outgoing CAN messages and one that checks if devices/SDOs have timed out (sanityCheck). 
- In synchronous mode, it is up to the user to call the BusManagers readMessagesSynchrounous(), writeMessagesSynchronous() and sanityCheckSynchronous() functions in his main loop.


To prevent overflow of the output buffer of the SocketCAN driver (which is used by the SocketBus class) there are two possible approaches:

- Set the output queue length to a value which is large enough to hold the messages of one cycle:
    ```sudo ip link set can0 txqueuelen 100```
- Setting the SocketBusOptions::sndBufLength_ to 1 (or any other small value > 0). This sets the socket buffer size to its minimal value and will make the socket blocking if this buffer is full (which is NOT the same as the buffer of the underlying netdevice)