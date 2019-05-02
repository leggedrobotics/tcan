#!/bin/bash

# Supported Bit-rates
BT_10KB="-s0"
BT_20KB="-s1"
BT_50KB="-s2"
BT_100KB="-s3"
BT_125KB="-s4"
BT_250KB="-s5"
BT_500KB="-s6"
BT_800KB="-s7"
BT_1MB="-s8"

#
# Helper functions
#

canif_start()
{
  echo "Attaching slcan device onto physical tty.."
  sudo slcand -o -f $3 -t hw -S 3000000 $1 $2
  sleep 0.05
  echo "Configuring the networkd device interface.."
  sudo ip link set $2 type can
  sleep 0.05
  echo "Setting $2 queue length to 1000..."
  sudo ip link set $2 txqueuelen 1000
  sleep 0.05
  echo "Bringing up $2 network interface..."
  sudo ip link set $2 up
  sleep 0.05
  echo "Setup complete."
}

canif_stop()
{
  echo "Shutting down the ${CANIF} interface."
  sudo ip link set $1 down
  sudo pkill -f "slcand .* $1"
  echo "Shutdown complete."
}

canif_kmstart()
{
  echo "Enabling CAN kernel driver modules.."
  sudo modprobe can
  sudo modprobe can-raw
  sudo modprobe slcan
  echo "Done."
}

canif_kmstop()
{
  echo "Disabling CAN kernel driver modules.."
  sudo rmmod slcan
  sudo rmmod can-raw
  sudo rmmod can
  echo "Done."
}

canif_info()
{
  ifconfig $1
}

canif_stats()
{
  ip -details -statistics link show $1
}

canif_usage()
{
  echo "Usage: $0 {start|stop|restart} <dev> <iface> <bit_rate>"
  echo "  OR   $0 {info|stats} <iface>"
  echo "  OR   $0 {kmstrt|kmstp}"
  echo "ARGUMETNS:"
  echo "  <dev>       [STRING]  The physical CANUSB device's mapping in /dev"
  echo "              Example value is '/dev/ttyUSB0'".
  echo "  <iface>     [STRING]  The desired CAN interface. Use ifconfig to see if available"
  echo "              Example value is 'can0'".
  echo "  <bit_rate>  [INT-KB] The Bit-rate at which to run the CAN bus in Kbps."
  echo "              Example value is '1000', for 1Mbps CAN.".
  echo "COMMANDS:"
  echo "  start    Enable and start the CAN device interface."
  echo "  stop     Stop and disable the CAN device interface."
  echo "  restart  Restart the CAN device interface."
  echo "  info     Show interface configuration."
  echo "  stats    Show interface statistics."
  echo "  kmstart  Load and start kernel drivers required for CAN."
  echo "  kmstop   Stop and remove all kernel drivers required for CAN."
  echo "EXAMPLES:"
  echo "  canif kmstart"
  echo "  canif /dev/ttyUSB0 can0 1000"
}

# canif_test()
# {
#
# }

# Set the specified command
CMD=$1

# First check first for kernel driver commands
case "${CMD}" in
  kmstart)
    canif_kmstart
    exit 0
  ;;

  kmstop)
    canif_kmstop
    exit 0
  ;;
esac

# Check for second argument, could be interface or device
if [ -z $2 ]
then
  echo "ERROR: Invalid arguments."
  canif_usage
  exit -1;
fi

# Secondly check for existing interface commands
case "${CMD}" in
  info)
    canif_info $2
    exit 0
  ;;

  stats)
    canif_stats $2
    exit 0
  ;;
esac

# Check for CAN interface argument
if [ -z $3 ]
then
  echo "ERROR: CAN interface not specified"
  canif_usage
  exit -1;
fi

# Check for bit-rate argument
if [ -z $4 ]
then
  echo "ERROR: Bit-rate not specified."
  canif_usage
  exit -1;
fi

# Set device and interface
CANDEV=$2
CANIF=$3

# Check for valid bit-rate
if [ $4 -eq 1000 ]
then
  echo "Bit-rate is set to 1Mbps"
  BT=${BT_1MB}
elif [ $4 -eq 800 ]
then
  echo "Bit-rate is set to 800Kbps"
  BT=${BT_800KB}
elif [ $4 -eq 500 ]
then
  echo "Bit-rate is set to 5000Kbps"
  BT=${BT_500KB}
elif [ $4 -eq 250 ]
then
  echo "Bit-rate is set to 250Kbps"
  BT=${BT_250KB}
elif [ $4 -eq 125 ]
then
  echo "Bit-rate is set to 125Kbps"
  BT=${BT_125KB}
elif [ $4 -eq 100 ]
then
  echo "Bit-rate is set to 100Kbps"
  BT=${BT_100KB}
elif [ $4 -eq 50 ]
then
  echo "Bit-rate is set to 50Kbps"
  BT=${BT_50KB}
elif [ $4 -eq 20 ]
then
  echo "Bit-rate is set to 20Kbps"
  BT=${BT_20KB}
elif [ $4 -eq 10 ]
then
  echo "Bit-rate is set to 10Kbps"
  BT=${BT_10KB}
else
  echo "ERROR: Invalid bit-rate."
  canif_usage
  exit -1
fi

#
# Main script
#
case "${CMD}" in
  start)
    echo "Starting up SLCAN device ${CANIF} via ${CANDEV}"
    canif_start ${CANDEV} ${CANIF} ${BT}
  ;;

  stop)
    echo "Shutting down SLCAN device ${CANIF}"
    canif_stop ${CANIF}
  ;;

  restart)
    echo "Starting up SLCAN device ${CANIF} via ${CANDEV}"
    canif_stop ${CANIF}
    canif_start ${CANDEV} ${CANIF} ${BT}
  ;;

  *)
    canif_usage
  ;;
esac
