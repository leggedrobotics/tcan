#!/bin/sh


do_start() {
    sudo modprobe vcan
    sudo ip link add dev $1 type vcan
    sudo ip link set $1 up
}


do_stop() {
    sudo ip link set $1 down
    sudo ip link delete $1
}



case "$1" in
    start)
        do_start $2
    ;;

    stop)
        do_stop $2
    ;;

    restart)
        do_stop $2
        do_start $2
    ;;

    *)
        echo "Usage: $0 {start|stop|restart} <name>"
        echo "   where name is the name of the interface (e.g. can0)"
    ;;
esac
