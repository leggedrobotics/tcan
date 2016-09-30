/*
 * Bus.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include <fcntl.h> // open(..) etc.
#include <termios.h> // tcgettatr
#include <unistd.h>
#include <sys/ioctl.h>

#include "tcan/UniversalSerialBus.hpp"
#include "message_logger/message_logger.hpp"

namespace tcan {

UniversalSerialBus::UniversalSerialBus(UniversalSerialBusOptions* options):
    Bus<UsbMsg>(options),
    fileDescriptor_(0)
{

}

UniversalSerialBus::~UniversalSerialBus()
{
    close(fileDescriptor_);
}

bool UniversalSerialBus::sanityCheck() {
    bool allFine = true;

    // todo: check timeouts

    isOperational_ = allFine;
    return allFine;
}


bool UniversalSerialBus::initializeInterface() {
    fileDescriptor_ = open(options_->name_.c_str(), O_RDWR | O_NDELAY); // open for read and write
    if (fileDescriptor_ < 0) {
        MELO_ERROR("Failed to open USB device: %s", options_->name_.c_str());
        return false;
    }

    // flushing is to be done after opening
    tcflush(fileDescriptor_, TCIOFLUSH);

    configureInterface();

    return true;
}

bool UniversalSerialBus::readData() {
    return true;
}

bool UniversalSerialBus::writeData(const UsbMsg& msg) {
    return true;
}


void UniversalSerialBus::configureInterface()
{
    const UniversalSerialBusOptions* options = static_cast<const UniversalSerialBusOptions*>(options_);

    struct termios newtio;
    //   memset(&newtio, 0, sizeof(newtio));
    if (tcgetattr(fileDescriptor_, &newtio)!=0) {
        MELO_ERROR("tcgetattr() 3 failed");
    }

    speed_t _baud=0;
    switch (options->baudrate)
    {
#ifdef B0
        case      0: _baud=B0;     break;
#endif

#ifdef B50
        case     50: _baud=B50;    break;
#endif
#ifdef B75
        case     75: _baud=B75;    break;
#endif
#ifdef B110
        case    110: _baud=B110;   break;
#endif
#ifdef B134
        case    134: _baud=B134;   break;
#endif
#ifdef B150
        case    150: _baud=B150;   break;
#endif
#ifdef B200
        case    200: _baud=B200;   break;
#endif
#ifdef B300
        case    300: _baud=B300;   break;
#endif
#ifdef B600
        case    600: _baud=B600;   break;
#endif
#ifdef B1200
        case   1200: _baud=B1200;  break;
#endif
#ifdef B1800
        case   1800: _baud=B1800;  break;
#endif
#ifdef B2400
        case   2400: _baud=B2400;  break;
#endif
#ifdef B4800
        case   4800: _baud=B4800;  break;
#endif
#ifdef B7200
        case   7200: _baud=B7200;  break;
#endif
#ifdef B9600
        case   9600: _baud=B9600;  break;
#endif
#ifdef B14400
        case  14400: _baud=B14400; break;
#endif
#ifdef B19200
        case  19200: _baud=B19200; break;
#endif
#ifdef B28800
        case  28800: _baud=B28800; break;
#endif
#ifdef B38400
        case  38400: _baud=B38400; break;
#endif
#ifdef B57600
        case  57600: _baud=B57600; break;
#endif
#ifdef B76800
        case  76800: _baud=B76800; break;
#endif
#ifdef B115200
        case 115200: _baud=B115200; break;
#endif
#ifdef B128000
        case 128000: _baud=B128000; break;
#endif
#ifdef B230400
        case 230400: _baud=B230400; break;
#endif
#ifdef B460800
        case 460800: _baud=B460800; break;
#endif
#ifdef B576000
        case 576000: _baud=B576000; break;
#endif
#ifdef B921600
        case 921600: _baud=B921600; break;
#endif
        default:
            //   case 256000:
            //      _baud=B256000;
            break;
    }
    cfsetospeed(&newtio, (speed_t)_baud);
    cfsetispeed(&newtio, (speed_t)_baud);

    /* We generate mark and space parity ourself. */
    int databits = options->databits;
    if (databits == 7 && (options->parity=="Mark" || options->parity == "Space"))
    {
        databits = 8;
    }
    switch (databits)
    {
        case 5:
            newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS5;
            break;
        case 6:
            newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS6;
            break;
        case 7:
            newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS7;
            break;
        case 8:
        default:
            newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8;
            break;
    }
    newtio.c_cflag |= CLOCAL | CREAD;

    //parity
    newtio.c_cflag &= ~(PARENB | PARODD);
    if (options->parity == "Even")
    {
        newtio.c_cflag |= PARENB;
    }
    else if (options->parity== "Odd")
    {
        newtio.c_cflag |= (PARENB | PARODD);
    }

    //hardware handshake
    /*   if (hardwareHandshake)
     newtio.c_cflag |= CRTSCTS;
     else
     newtio.c_cflag &= ~CRTSCTS;*/
    newtio.c_cflag &= ~CRTSCTS;

    //stopbits
    if (options->stopbits==2)
    {
        newtio.c_cflag |= CSTOPB;
    }
    else
    {
        newtio.c_cflag &= ~CSTOPB;
    }

    //   newtio.c_iflag=IGNPAR | IGNBRK;
    newtio.c_iflag=IGNBRK;
    //   newtio.c_iflag=IGNPAR;

    //software handshake
    if (options->softwareHandshake)
    {
        newtio.c_iflag |= IXON | IXOFF;
    }
    else
    {
        newtio.c_iflag &= ~(IXON|IXOFF|IXANY);
    }

    newtio.c_lflag=0;
    newtio.c_oflag=0;

    newtio.c_cc[VTIME]=1;
    newtio.c_cc[VMIN]=60;

    //   tcflush(m_fd, TCIFLUSH);
    if (tcsetattr(fileDescriptor_, TCSANOW, &newtio)!=0)
    {
        MELO_ERROR("tcsetattr() 1 failed");
    }

    int mcs=0;
    ioctl(fileDescriptor_, TIOCMGET, &mcs);
    mcs |= TIOCM_RTS;
    ioctl(fileDescriptor_, TIOCMSET, &mcs);

    if (tcgetattr(fileDescriptor_, &newtio)!=0)
    {
        MELO_ERROR("tcsetattr() 4 failed");
    }

    //hardware handshake
    if (options->hardwareHandshake)
    {
        newtio.c_cflag |= CRTSCTS;
    }
    else
    {
        newtio.c_cflag &= ~CRTSCTS;
    }
    /*  if (on)
     newtio.c_cflag |= CRTSCTS;
  else
     newtio.c_cflag &= ~CRTSCTS;*/
    if (tcsetattr(fileDescriptor_, TCSANOW, &newtio)!=0)
    {
        MELO_ERROR("tcsetattr() 2 failed");
    }
}

} /* namespace tcan */

