/*
 * SocketBus.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include "tcan/SocketBus.hpp"

#include "message_logger/message_logger.hpp"

namespace tcan {

SocketBus::SocketBus(const std::string& interface):
    SocketBus(new SocketBusOptions(interface))
{
}

SocketBus::SocketBus(SocketBusOptions* options):
    CanBus(options),
    socket_()
{
}

SocketBus::~SocketBus()
{
    stopThreads();
    close(socket_.fd);
}

bool SocketBus::initializeInterface()
{
    const SocketBusOptions* options = static_cast<const SocketBusOptions*>(options_);
    const char* interface = options->name_.c_str();

    /* open socket */
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(fd < 0) {
        MELO_FATAL("Opening CAN channel %s failed: %d", interface, fd);
        return false;
    }


    /* configure socket */
    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface);
    ioctl(fd, SIOCGIFINDEX, &ifr);

    // loopback
    int loopback = options->loopback_;
    if(setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) != 0) {
        MELO_WARN("Failed to set loopback mode");
        perror("setsockopt");
    }

    // receive own messages
    int recv_own_msgs = 0; /* 0 = disabled (default), 1 = enabled */
    if(setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) != 0) {
    	MELO_WARN("Failed to set reception of own messages option:\n  %s", strerror(errno));
    }

    // CAN error handling
    can_err_mask_t err_mask = options->canErrorMask_;
    if(setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) != 0) {
    	MELO_WARN("Failed to set error mask:\n  %s", strerror(errno));
    }

    // get default bufer sizes
    //	int buf_size;
    //	socklen_t len;
    //	getsockopt(fd, SOL_SOCKET, SO_SNDBUF, &buf_size, &len);
    //	printf("sndbuf size: %d (%d)\n", buf_size, len);
    //
    //	getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &buf_size, &len);
    //	printf("rcvbuf size: %d (%d)\n", buf_size, len);

    // On some CAN drivers, the txqueuelen of the netdevice cannot be changed (default=10).
    // The buffer size of the socket is larger than those 10 messages, so the write(..) call never blocks but raises a ENOBUFS error when writing to the netdevice.
    // This renders poll(..) useless. Not the writes in the sockets are the limiting pipe but the writes in the underlying netdevice.
    // The recommended way to fix this is to increase the txqueuelen of the netdevice (e.g. command line: ip link set can0 txqueuelen 1000)
    // If this is not possible, the sndbuf size of the socket can be decrease, such that the socket writes are the limiting pipe, leading to blocking socket write(..) calls. (write becomes poll(..)-able)
    // https://www.mail-archive.com/socketcan-users@lists.berlios.de/msg00787.html
    // http://socket-can.996257.n3.nabble.com/Solving-ENOBUFS-returned-by-write-td2886.html
    if(options->sndBufLength_ != 0) {
        if(setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &(options->sndBufLength_), sizeof(options->sndBufLength_)) != 0) {
        	MELO_WARN("Failed to set sndBuf length:\n  %s", strerror(errno));
        }
    }

    // set read timeout
    timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    if(setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout)) != 0) {
        MELO_WARN("Failed to set read timeout:\n  %s", strerror(errno));
    }
    // set write timeout
    if(setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout)) != 0) {
        MELO_WARN("Failed to set write timeout:\n  %s", strerror(errno));
    }



    // set up filters
    if(options->canFilters_.size() != 0) {
        if(setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &(options->canFilters_[0]), sizeof(can_filter)*options->canFilters_.size()) != 0) {
            MELO_WARN("Failed to set CAN raw filters:\n  %s", strerror(errno));
        }
    }

    //Set nonblocking for synchronous mode
    if(!options_->asynchronous_) {
        int flags;
        if (-1 == (flags = fcntl(fd, F_GETFL, 0))) flags = 0;
        if(fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
            MELO_ERROR("Failed to set socket nonblocking:\n  %s", strerror(errno));
        }
    }

    /* bind socket */
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(struct sockaddr_can));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        MELO_FATAL("Error in socket %s bind:\n  %s", interface, strerror(errno));
        return false;
    }

    socket_ = {fd, POLLOUT, 0};

    MELO_INFO("Opened socket %s.", interface);

    return true;
}


bool SocketBus::readData() {

    // no need to poll the socket.
    // In synchronous mode, the socket is non-blocking, so this function returns as soon as there is no data available to be read
    // If asynchronous, we set the socket to blocking and have a separate thread reading from it.

    can_frame frame;
    const int bytes_read = read( socket_.fd, &frame, sizeof(struct can_frame));
    //	printf("CanManager_ bytes read: %i\n", bytes_read);

    if(bytes_read<=0) {
        //		printf("read nothing\n");
        return false;
    } else {
        //		printf("CanManager:bus_routine: Data received from iBus %i, n. Bytes: %i \n", iBus, bytes_read);

        handleMessage( CanMsg(frame.can_id, frame.can_dlc, frame.data) );
    }

    return true;
}


bool SocketBus::writeData(const CanMsg& cmsg) {

    // poll the socket only in synchronous mode, so this function DOES block until socket is writable (or timeout), even if the socket is non-blocking.
    // If asynchronous, we set the socket to blocking and have a separate thread writing to it.

    if(!options_->asynchronous_) {
        socket_.revents = 0;

        const int ret = poll( &socket_, 1, 1000 );

        if ( ret == -1 ) {
            MELO_ERROR("poll failed on bus %s:\n  %s", options_->name_.c_str(), strerror(errno));
            return false;
        }else if ( ret == 0 || !(socket_.revents & POLLOUT) ) {
            // poll timed out, without being able to write => raise error
            MELO_WARN("polling for socket writeability timed out for bus %s. Socket overflow?", options_->name_.c_str());
            return false;
        }else{
            // socket ready for write operations => proceed
        }
    }

    can_frame frame;
    frame.can_id = cmsg.getCobId();
    frame.can_dlc = cmsg.getLength();
    std::copy(cmsg.getData(), &(cmsg.getData()[frame.can_dlc]), frame.data);

    int ret;
    if( ( ret = write(socket_.fd, &frame, sizeof(struct can_frame)) ) != sizeof(struct can_frame)) {
        MELO_ERROR("Error at sending CAN message %x on bus %s (return value=%d):\n  %s", cmsg.getCobId(), options_->name_.c_str(), ret, strerror(errno));
        return false;
    }

    return true;
}

void SocketBus::handleBusError(const CanMsg& msg) {
    const auto& cob = msg.getCobId();
    const auto value = msg.getData();

    MELO_ERROR("received bus error frame:");
    // cob id
    if(cob & CAN_ERR_TX_TIMEOUT) {
        MELO_ERROR("  TX timeout (by netdevice driver)");
    }
    if(cob & CAN_ERR_LOSTARB) {
        MELO_ERROR("  lost arbitration");
    }
    if(cob & CAN_ERR_CRTL) {
        MELO_ERROR("  controller problems");
    }
    if(cob & CAN_ERR_PROT) {
        MELO_ERROR("  protocol violations");
    }
    if(cob & CAN_ERR_TRX) {
        MELO_ERROR("  transceiver status");
    }
    if(cob & CAN_ERR_ACK) {
        MELO_ERROR("  received no ACK on transmission");
    }
    if(cob & CAN_ERR_BUSOFF) {
        MELO_ERROR("  bus off");
    }
    if(cob & CAN_ERR_BUSOFF) {
        MELO_ERROR("  bus error (may flood!)");
    }
    if(cob & CAN_ERR_RESTARTED) {
        MELO_ERROR("  controller restarted");
    }

    // bit 0
    MELO_ERROR("    bit number in bitstream: %d", value[0]);

    // bit 1
    switch(value[1]) {
        default: // fall-through!
        case CAN_ERR_CRTL_UNSPEC:
            MELO_ERROR("    error status of CAN-controller: unspecified");
            break;

        case CAN_ERR_CRTL_RX_OVERFLOW:
            MELO_ERROR("    error status of CAN-controller: rx buffer overflow");
            break;

        case CAN_ERR_CRTL_TX_OVERFLOW:
            MELO_ERROR("    error status of CAN-controller: tx buffer overflow");
            break;

        case CAN_ERR_CRTL_RX_WARNING:
            MELO_ERROR("    error status of CAN-controller: reached warning level for RX errors");
            break;

        case CAN_ERR_CRTL_TX_WARNING:
            MELO_ERROR("    error status of CAN-controller: reached warning level for TX errors");
            break;

        case CAN_ERR_CRTL_RX_PASSIVE:
            MELO_ERROR("    error status of CAN-controller: reached error passive status RX");
            break;

        case CAN_ERR_CRTL_TX_PASSIVE:
            MELO_ERROR("    error status of CAN-controller: reached error passive status TX");
            break;
    }

    // bit 2
    switch(value[2]) {
        default: // fall-through!
        case CAN_ERR_PROT_UNSPEC:
            MELO_ERROR("    error in CAN protocol (type): unspecified");
            break;

        case CAN_ERR_PROT_BIT:
            MELO_ERROR("    error in CAN protocol (type): single bit error");
            break;

        case CAN_ERR_PROT_FORM:
            MELO_ERROR("    error in CAN protocol (type): frame format error");
            break;

        case CAN_ERR_PROT_STUFF:
            MELO_ERROR("    error in CAN protocol (type): bit stuffing error");
            break;

        case CAN_ERR_PROT_BIT0:
            MELO_ERROR("    error in CAN protocol (type): unable to send dominant bit");
            break;

        case CAN_ERR_PROT_BIT1:
            MELO_ERROR("    error in CAN protocol (type): unable to send recessive bit");
            break;

        case CAN_ERR_PROT_OVERLOAD:
            MELO_ERROR("    error in CAN protocol (type): bus overload");
            break;

        case CAN_ERR_PROT_ACTIVE:
            MELO_ERROR("    error in CAN protocol (type): active error announcement");
            break;

        case CAN_ERR_PROT_TX:
            MELO_ERROR("    error in CAN protocol (type): error occurred on transmission");
            break;
    }

    // bit 3
    switch(value[3]) {
        default:
        case CAN_ERR_PROT_LOC_UNSPEC:
            MELO_ERROR("    error in CAN protocol (location): unspecified");
            break;

        case CAN_ERR_PROT_LOC_SOF:
            MELO_ERROR("    error in CAN protocol (location): start of frame");
            break;

        case CAN_ERR_PROT_LOC_ID28_21:
            MELO_ERROR("    error in CAN protocol (location): ID bits 28 - 21 (SFF: 10 - 3)");
            break;

        case CAN_ERR_PROT_LOC_ID20_18:
            MELO_ERROR("    error in CAN protocol (location): ID bits 20 - 18 (SFF: 2 - 0 )");
            break;

        case CAN_ERR_PROT_LOC_SRTR:
            MELO_ERROR("    error in CAN protocol (location): substitute RTR (SFF: RTR)");
            break;

        case CAN_ERR_PROT_LOC_IDE:
            MELO_ERROR("    error in CAN protocol (location): identifier extension");
            break;

        case CAN_ERR_PROT_LOC_ID17_13:
            MELO_ERROR("    error in CAN protocol (location): ID bits 17-13");
            break;

        case CAN_ERR_PROT_LOC_ID12_05:
            MELO_ERROR("    error in CAN protocol (location): ID bits 12-5");
            break;

        case CAN_ERR_PROT_LOC_ID04_00:
            MELO_ERROR("    error in CAN protocol (location): ID bits 4-0");
            break;

        case CAN_ERR_PROT_LOC_RTR:
            MELO_ERROR("    error in CAN protocol (location): RTR");
            break;

        case CAN_ERR_PROT_LOC_RES1:
            MELO_ERROR("    error in CAN protocol (location): reserved bit 1");
            break;

        case CAN_ERR_PROT_LOC_RES0:
            MELO_ERROR("    error in CAN protocol (location): reserved bit 0");
            break;

        case CAN_ERR_PROT_LOC_DLC:
            MELO_ERROR("    error in CAN protocol (location): data length code");
            break;

        case CAN_ERR_PROT_LOC_DATA:
            MELO_ERROR("    error in CAN protocol (location): data section");
            break;

        case CAN_ERR_PROT_LOC_CRC_SEQ:
            MELO_ERROR("    error in CAN protocol (location): CRC sequence");
            break;

        case CAN_ERR_PROT_LOC_CRC_DEL:
            MELO_ERROR("    error in CAN protocol (location): CRC delimiter");
            break;

        case CAN_ERR_PROT_LOC_ACK:
            MELO_ERROR("    error in CAN protocol (location): ACK slot");
            break;

        case CAN_ERR_PROT_LOC_ACK_DEL:
            MELO_ERROR("    error in CAN protocol (location): ACK delimiter");
            break;

        case CAN_ERR_PROT_LOC_EOF:
            MELO_ERROR("    error in CAN protocol (location): end of frame");
            break;

        case CAN_ERR_PROT_LOC_INTERM:
            MELO_ERROR("    error in CAN protocol (location): intermission");
            break;
    }

    // bit 4
    switch(value[4]) {
        default:
        case CAN_ERR_TRX_UNSPEC:
            MELO_ERROR("    error status of CAN-transceiver: CAN_ERR_TRX_UNSPEC");
            break;

        case CAN_ERR_TRX_CANH_NO_WIRE:
            MELO_ERROR("    error status of CAN-transceiver: CAN_ERR_TRX_CANH_NO_WIRE");
            break;

        case CAN_ERR_TRX_CANH_SHORT_TO_BAT:
            MELO_ERROR("    error status of CAN-transceiver: CAN_ERR_TRX_CANH_SHORT_TO_BAT");
            break;

        case CAN_ERR_TRX_CANH_SHORT_TO_VCC:
            MELO_ERROR("    error status of CAN-transceiver: CAN_ERR_TRX_CANH_SHORT_TO_VCC");
            break;

        case CAN_ERR_TRX_CANH_SHORT_TO_GND:
            MELO_ERROR("    error status of CAN-transceiver: CAN_ERR_TRX_CANH_SHORT_TO_GND");
            break;

        case CAN_ERR_TRX_CANL_NO_WIRE:
            MELO_ERROR("    error status of CAN-transceiver: CAN_ERR_TRX_CANL_NO_WIRE");
            break;

        case CAN_ERR_TRX_CANL_SHORT_TO_BAT:
            MELO_ERROR("    error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_BAT");
            break;

        case CAN_ERR_TRX_CANL_SHORT_TO_VCC:
            MELO_ERROR("    error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_VCC");
            break;

        case CAN_ERR_TRX_CANL_SHORT_TO_GND:
            MELO_ERROR("    error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_GND");
            break;

        case CAN_ERR_TRX_CANL_SHORT_TO_CANH:
            MELO_ERROR("    error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_CANH");
            break;
    }

    // bit 5-7
    MELO_ERROR("    controller specific additional information: %x, %x, %x", value[5], value[6], value[7]);
}

} /* namespace tcan */
