#!/bin/bash

do_start() {
    sudo slcand -o $3 -t hw -S 3000000 $1 $2
    sleep 0.2
    sudo ip link set $2 txqueuelen $4
    sudo ip link set $2 up
}

do_stop() {
    sudo ip link set $1 down
    sudo pkill -f "slcand .* $1"
}

case "$1" in
    start)
        do_start $2 $3 $4 ${5:-1000}
    ;;

    stop)
        do_stop $2
    ;;

    restart)
        do_stop $3
        do_start $2 $3 $4 ${5:-1000}
    ;;

    *)
        echo "Usage: $0 {start|stop|restart} [<dev>] <name> [<can_rate>] [<txqueuelen>]"
        echo "    where dev is the file descriptor (e.g. /dev/ttyUSB0), name the name of"
        echo "    the interface (e.g. can0), can_rate one of the following:"
        echo "      -s8: 1Mbit"
        echo "      -s7: 800kbit"
        echo "      -s6: 500kbit"
        echo "      -s5: 250kbit"
        echo "      -s4: 125kbit"
        echo "      -s3: 100kbit"
        echo "      -s2: 50kbit"
        echo "      -s1: 10kbit"
        echo "    and txqueuelen the length of the transmit queue (default=1000)"
    ;;
esac
