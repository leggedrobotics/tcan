#pragma once

#include <termios.h> // tcgettatr
#include <memory>

#include "tcan/Bus.hpp"
#include "tcan_usb/UniversalSerialBusOptions.hpp"
#include "tcan_usb/UsbMsg.hpp"

namespace tcan_usb {

class UniversalSerialBus : public tcan::Bus<UsbMsg> {
 public:
    UniversalSerialBus() = delete;
    UniversalSerialBus(std::unique_ptr<UniversalSerialBusOptions>&& options);

    ~UniversalSerialBus() override;

    bool sanityCheck() override;

    int getPollableFileDescriptor() const override { return fileDescriptor_; }

protected:
    bool initializeInterface() override;
    bool readData() override;
    bool writeData(std::unique_lock<std::mutex>* lock) override;

 private:
    void configureInterface();

 private:
    int fileDescriptor_;

    termios savedAttributes_;

    unsigned int deviceTimeoutCounter_;
};

} /* namespace tcan_usb */
