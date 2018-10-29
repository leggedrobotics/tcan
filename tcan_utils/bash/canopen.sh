#!/bin/bash

# Interface device, default is according to SocketCAN standard
CANIF_DEFAULT="can0"
# Remote device node-id, default is for msg broadcast
NODEID_DEFAULT="00"

#
# Check arguments
#

if [ -z "$2" ]
then
  CANIF=${CANIF_DEFAULT}
else
  CANIF=$2
fi

if [ -z "$3" ]
then
  NODEID=${NODEID}
else
  NODEID=$3
fi

#
# Main script
#

case "$1" in
  sync)
    cansend ${CANIF} "080#"
  ;;

  preop)
    cansend ${CANIF} "000#80.${NODEID}"
  ;;

  op)
    cansend ${CANIF} "000#01.${NODEID}"
  ;;

  stop)
    cansend ${CANIF} "000#02.${NODEID}"
  ;;

  resnd)
    cansend ${CANIF} "000#81.${NODEID}"
  ;;

  rescm)
    cansend ${CANIF} "000#82.${NODEID}"
  ;;

  *)
    echo "Usage: $0 {sync|preop|op|stop|resnd|rescm} <dev> <nodeid>"
    echo "ARGUMETNS:"
    echo "  <dev>    [STRING]  The CAN interface device. Use ifconfig to see available"
    echo "           Default value is 'can0'".
    echo "  <nodeid> [HEX-INT] The Node-ID of the remote CANopen device in 2-digit Hexadecimal."
    echo "           Default value is '00', for broadcast to all slaves".
    echo "COMMANDS:"
    echo "  sync     Send a CANopen SYNC command on the bus."
    echo "  preop    Send an NMT command to send slave(s) to Pre-Operational state."
    echo "  op       Send an NMT command to send slave(s) to Operational state."
    echo "  stop     Send an NMT command to send slave(s) to Stopped state."
    echo "  resnd    Send an NMT command to reset slave node(s)."
    echo "  rescm    Send an NMT command to reset slave node(s) communication inteface."
    echo "EXAMPLE:"
    echo "  canopen op can0 05"
  ;;
esac
