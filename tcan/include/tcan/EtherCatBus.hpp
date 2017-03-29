/*
 * EtherCatBus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <stdint.h>
#include <unordered_map>
#include <memory>
#include <functional>
#include <vector>
#include <algorithm> // copy(..)
#include <inttypes.h>

#include <ethercat.h>


#include "tcan/Bus.hpp"
#include "tcan/CanMsg.hpp"
#include "tcan/EtherCatBusOptions.hpp"
#include "tcan/EtherCatDevice.hpp"
#include "tcan/EthernetFrame.hpp"

namespace tcan {



struct EthernetHeader {
    uint8_t destination_[6];
    uint8_t source_[6];
    uint8_t etherType_[2];

    EthernetHeader()
    :   destination_{0,0,0,0,0,0},
        source_{0,0,0,0,0,0},
        etherType_{0,0} {}
};

struct EthercatHeader {
    uint16_t data_ = 0;

    EthercatHeader() {}

    uint16_t getLength() const {
        return (data_ & 0xFFE0) >> 5;
    }

    void setLength(uint16_t length) {
        data_ &= ~0xFFE0; // clear bits first.
        data_ = (length << 5) & 0xFFE0;
    }

    uint16_t getReserved() const {
        return (data_ & 0x0001) >> 4;
    }

    void setReserved(uint16_t reserved) {
        data_ &= ~0x0001; // clear bits first.
        data_ = (reserved << 4) & 0x0001;
    }

    uint16_t getType() const {
        return (data_ & 0x000F) >> 0;
    }

    void setType(uint16_t type) {
        data_ &= ~0x000F; // clear bits first.
        data_ = (type << 0) & 0x000F;
    }
};

class Datagram {
 public:
    inline void resize(const uint16_t length) {
        uint8_t* oldData = data_;
        data_ = new uint8_t[length];
        std::copy(&oldData[0], &oldData[header_.getLength()], data_);
        header_.setLength(length);
        delete[] oldData;
    }

    template <typename T>
    inline void write(const uint16_t memoryPosition, const T& data) {
        // check if memoryPosition lies within data_?
        std::copy(&data_[memoryPosition], &data_[memoryPosition + sizeof(T)], &data);
    }

    inline const uint16_t getTotalLength() const { return getDataLength() + 12; }
    inline const uint16_t getDataLength() const { return header_.getLength(); }
    inline const uint8_t* getData() const { return data_; }
    inline const uint16_t getWorkingCounter() const { return workingCounter_; }

 private:

    struct DatagramHeader {
        enum class Command : uint8_t {
            NOP=0, // No operation
            APRD=1, // Auto Increment Read
            APWR=2, // Auto Increment Write
            APRW=3, // Auto Increment Read Write
            FPRD=4, // Configured Address Read
            FPWR=5, // Configured Address Write
            FPRW=6, // Configured Address Read Write
            BRD=7, // Broadcast Read
            BWR=8, // Broadcast Write
            BRW=9, // Broadcast Read Write
            LRD=10, // Logical Memory Read
            LWR=11, // Logical Memory Write
            LRW=12, // Logical Memory Read Write
            ARMW=13, // Auto Increment Read Multiple Write
            FRMW=14 // Configured Read Multiple Write
        };

        Command cmd_;
        uint8_t idx_;
        uint32_t address_;
//        uint16_t len_           :11; // does this sub-byte work? -> Remo: I do not think so, since the order is compiler dependent
//        uint16_t reserved_      :3;
//        uint16_t circulating_   :1;
//        uint16_t more_          :1;
        uint16_t lenRCM_;
        uint16_t irq_;

//        DatagramHeader(): cmd_(Command::NOP), idx_(0), address_(0), len_(0), reserved_(0), circulating_(0), more_(0), irq_(0) { }
        DatagramHeader(): cmd_(Command::NOP), idx_(0), address_(0), lenRCM_(0), irq_(0) { }

        uint16_t getLength() const {
            return (lenRCM_ & 0xFFE0) >> 5;
        }

        void setLength(uint16_t length) {
            lenRCM_ &= ~0xFFE0; // clear bits first.
            lenRCM_ = (length << 5) & 0xFFE0;
        }

        uint16_t getReserved() const {
            return (lenRCM_ & 0x001C) >> 2;
        }

        void setReserved(uint16_t reserved) {
            lenRCM_ &= ~0x001C; // clear bits first.
            lenRCM_ = (reserved << 2) & 0x001C;
        }

        uint16_t getCirculating() const {
            return (lenRCM_ & 0x0002) >> 1;
        }

        void setCirculating(uint16_t circulating) {
            lenRCM_ &= ~0x0002; // clear bits first.
            lenRCM_ = (circulating << 1) & 0x0002;
        }

        uint16_t getMore() const {
            return (lenRCM_ & 0x0001) >> 0;
        }

        void setMore(uint16_t more) {
            lenRCM_ &= ~0x0001; // clear bits first.
            lenRCM_ = (more << 1) & 0x0001;
        }
    }; //__attribute__((packed)); // prevent structure padding


 public:
    DatagramHeader header_;
    uint8_t* data_ = nullptr;
    uint16_t workingCounter_ = 0;

    // have a map of (name string => memory address) to have human-readable access to the fields in data_? would need to update this on resize(..) calls..
};

class EtherCatBus : public Bus<EthernetFrame> {
 public:
    typedef std::function<bool(const CanMsg&)> CallbackPtr;
    typedef std::unordered_map<uint32_t, std::pair<EtherCatDevice*, CallbackPtr>> AddressToFunctionMap;

    EtherCatBus() = delete;
    EtherCatBus(EtherCatBusOptions* options)
    : Bus<EthernetFrame>(options),
      wkcExpected_(0),
      wkc_(0),
      needlf_(false),
      inOP_(false) {}

    virtual ~EtherCatBus() {
      ec_close();
      MELO_INFO_STREAM("Closed socket.");
    }

    /*!
     * in-place construction of a new device
     * @param options   pointer to the option class of the device
     * @return true if successful
     */
    template <class C, typename TOptions>
    inline std::pair<C*, bool> addDevice(TOptions* options) {
        C* dev = new C(options);
        bool success = addDevice(dev);
        return std::make_pair(dev, success);
    }

    /*! Adds a device to the device vector and calls its initDevice function
     * @param device    Pointer to the device
     * @return true if init was successful
     */
    inline bool addDevice(EtherCatDevice* device) {
        // asssign the device some id to calculate the offset in ethernet frame address
        devices_.push_back(device);
        return device->initDeviceInternal(this);
    }

    template <class T>
    inline std::shared_ptr<Datagram> addReadDatagram(Datagram&& datagram, T* device, bool(std::common_type<T>::type::*fp)()) {
        // add a datagram linked with a callback to addressToFunctionMap_, use datagram address as key
        return addDatagram(std::forward<Datagram>(datagram));
    }

    inline std::shared_ptr<Datagram> addWriteDatagram(Datagram&& datagram) {
        return addDatagram(std::forward<Datagram>(datagram));
    }

    /*!
     * Should be called from the same thread as write operations on the datagrams, otherwise it is up to the user to ensure thread safety
     * @return  true on success
     */
    inline void dispatchFrame() {
        // TODO: How are SDOs handled?
        // TODO: Is the ethernet frame size fixed?
        // TODO: SOEM: Where are the headers? First slave inputs (RxPDOs), the outputs (TxPDOs))

        // TODO: These values are still unknown.
        EthernetHeader ethernetHeader;
        EthercatHeader ethercatHeader;
        uint64_t fcs = 0;

        // Compute length of the ethernet frame.
        uint16_t len = 20; // Ethernet + EtherCat overhead. TODO: derive EthernetFrame from GenericMsg and add length getter
        for (const auto& datagramPtr : datagrams_) {
            len += datagramPtr->getTotalLength();
        }

        // Instantiate data.
        uint8_t* data = new uint8_t[len];
        uint16_t pos = 0;

        // Copy ethernet header.
        for (uint16_t i = 0; i < 6; i++) {
            data[pos++] = ethernetHeader.destination_[i];
        }
        for (uint16_t i = 0; i < 6; i++) {
            data[pos++] = ethernetHeader.source_[i];
        }
        for (uint16_t i = 0; i < 2; i++) {
            data[pos++] = ethernetHeader.etherType_[i];
        }
        // Copy ethercat header.
        data[pos++] = static_cast<uint8_t>((ethercatHeader.data_ >> 0) & 0xFF);
        data[pos++] = static_cast<uint8_t>((ethercatHeader.data_ >> 8) & 0xFF);
        // Copy ethercat datagrams.
        for (const auto& datagramPtr : datagrams_) {
            // Copy ethercat datagram header.
            data[pos++] = static_cast<uint8_t>(datagramPtr->header_.cmd_);
            data[pos++] = static_cast<uint8_t>(datagramPtr->header_.idx_);
            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.address_ >> 0) & 0xFF);
            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.address_ >> 8) & 0xFF);
            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.address_ >> 16) & 0xFF);
            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.address_ >> 24) & 0xFF);
            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.lenRCM_ >> 0) & 0xFF);
            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.lenRCM_ >> 8) & 0xFF);
            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.irq_ >> 0) & 0xFF);
            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.irq_ >> 8) & 0xFF);
            // Copy ethercat datagram data.
            for (uint16_t i = 0; i < datagramPtr->getDataLength(); i++) {
                data[pos++] = static_cast<uint8_t>(datagramPtr->data_[i]);
            }
            // Copy ethercat datagram working counter.
            data[pos++] = static_cast<uint8_t>((datagramPtr->workingCounter_ >> 0) & 0xFF);
            data[pos++] = static_cast<uint8_t>((datagramPtr->workingCounter_ >> 8) & 0xFF);
        }
        // Copy ethernet FCS.
        for(uint16_t i = 0; i < 4; i++) {
            data[pos++] = static_cast<uint8_t>((fcs >> i*8) & 0xFF);
        }

        // compute expected working counter

        // other option:
        EthernetFrame msg;
        msg.emplaceData(len, data);
        sendMessage(msg);
    }

    void discoverDevices() {
        // do not add the devices by hand but get the topology and memory from the connected devices?
    }

 public:/// INTERNAL FUNCTIONS
    /*! Is called after reception of a message. Routes the message to the callback.
     * @param cmsg    reference to the can message
     */
    void handleMessage(const EthernetFrame& msg) {
        // check time the frame took to travel through the physical bus
        // check working counters
        // extract datagrams from ethernet frame and map datagrams to device callback
    }

    /*! Initialize the device driver
     * @return true if successful
     */
    virtual bool initializeInterface() {
        printf("Starting simple test\n");
        needlf_ = false;
        inOP_ = false;

        /*
         * Followed by start of the application we need to set up the NIC to be used as
         * EtherCAT Ethernet interface. In a simple setup we call ec_init(ifname) and if
         * SOEM comes with support for cable redundancy we call ec_init_redundant that
         * will open a second port as backup. You can send NULL as ifname if you have a
         * dedicated NIC selected in the nicdrv.c. It returns >0 if succeeded.
         */
        const char* ifname = options_->name_.c_str();
        if (ec_init(ifname) <= 0) {
            MELO_ERROR_STREAM("No socket connection on '" << ifname << "'.");
            MELO_ERROR_STREAM("Excecute as root.");
            return false;
        }

        MELO_INFO_STREAM("EtherCAT initialization on '" << ifname << "' succeeded.");

        /*
         * SOEM is a light weight ethercat master library used in embedded systems,
         * It supports only runtime configuration. It requests a BRD (Broad Cast Read)
         * of address 0, all fully functional slaves in the network will respond to this
         * request, and therefore we will get a working counter equal to the number of
         * slaves in the network. ec_config_init also sets up the mailboxes for slaves
         * that support it. When ec_config_init finishes it will have requested all
         * slaves to state PRE_OP. All data read and configured are stored in a global
         * array which acts as a placeholder for key values, consult ec_slave for
         * detailed information.
         */
        if (ec_config_init(FALSE) == 0) {
            MELO_ERROR_STREAM("No slaves have been found.");
            return false;
        }

        MELO_INFO_STREAM("The following " << ec_slavecount << " slaves have been found and configured:");
        for (int i = 1; i <= ec_slavecount; i++) {
            MELO_INFO_STREAM("- " << std::string(ec_slave[i].name));
        }

        if (!allDevicesFound()) {
            MELO_ERROR_STREAM("Not all expected slaves have been found.");
            return false;
        }

        /*!
         * We now have the network up and configured. Mailboxes are up for slaves that
         * support it. Next we will create an IOmap and configure the SyncManager's and
         * FMMU's to link the EtherCAT master and the slaves. The IO mapping is done
         * automatically, SOEM strives to keep the logical process image as compact as
         * possible. It is done by trying to fit Bit oriented slaves together in single
         * bytes. Below is an example of 8 slaves and how they are ordered. During
         * mapping SOEM also calculates an expected WKC for the IO mapped together.
         * That is the primary key to detect errors.
         *
         *  * Outputs are placed together in the beginning of the IOmap
         *  * Inputs follow
         *
         * When the mapping is done SOEM requests slaves to enter SAFE_OP.
         */
        ec_config_map(&IOmap_);

        ec_configdc(); // TODO what is this doing?

        printf("Slaves mapped, state to SAFE_OP.\n");
        /* wait for all slaves to reach SAFE_OP state */
        ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

        int oloop = ec_slave[0].Obytes;
        if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
        if (oloop > 8) oloop = 8;
        int iloop = ec_slave[0].Ibytes;
        if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
        if (iloop > 8) iloop = 8;

        printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

        printf("Request operational state for all slaves\n");
        wkcExpected_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        printf("Calculated workcounter %d\n", wkcExpected_.load());
        ec_slave[0].state = EC_STATE_OPERATIONAL;
        /* send one valid process data to make outputs in slaves happy*/
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        /* request OP state for all slaves */
        ec_writestate(0);
        int chk = 40;
        /* wait for all slaves to reach OP state */
        do
        {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
        }
        while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
        if (ec_slave[0].state == EC_STATE_OPERATIONAL )
        {
            printf("Operational state reached for all slaves.\n");
            inOP_ = true;
            /* cyclic loop */
            for(int i = 1; i <= 10000; i++)
            {
                ec_send_processdata();
                wkc_ = ec_receive_processdata(EC_TIMEOUTRET);

                if(wkc_ >= wkcExpected_)
                {
                    printf("Processdata cycle %4d, WKC %d , O:", i, wkc_.load());

                    for(int j = 0 ; j < oloop; j++)
                    {
                        printf(" %2.2x", *(ec_slave[0].outputs + j));
                    }

                    printf(" I:");
                    for(int j = 0 ; j < iloop; j++)
                    {
                        printf(" %2.2x", *(ec_slave[0].inputs + j));
                    }
                    printf(" T:%" PRId64 "\r",ec_DCtime);
                    needlf_ = true;
                }
                osal_usleep(5000);

            }
            inOP_ = false;
        }
        else
        {
            printf("Not all slaves reached operational state.\n");
            ec_readstate();
            for(int i = 1; i<=ec_slavecount ; i++)
            {
                if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                {
                    printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                }
            }
        }
        printf("\nRequest init state for all slaves\n");
        ec_slave[0].state = EC_STATE_INIT;
        /* request INIT state for all slaves */
        ec_writestate(0);


        return true;
    }

    /*! read CAN message from the device driver
     * @return true if a message was successfully read and parsed
     */
    virtual bool readData() {
        if (!inOP_) {
          MELO_WARN_STREAM("Devices are not in OP.");
          return false;
        }
        wkc_ = ec_receive_processdata(EC_TIMEOUTRET);
        MELO_INFO_STREAM("Working counter is at " << wkc_ << "/" << wkcExpected_ << ".");
        if (wkc_ < wkcExpected_) {
          MELO_WARN_STREAM("Working counter is too low.");
          return false;
        }
        EthernetFrame ethernetFrame;
        handleMessage(ethernetFrame);
        return true;
    }

    /*! write CAN message to the device driver
     * @return true if the message was successfully written
     */
    virtual bool writeData(const EthernetFrame& msg) {
        if (!inOP_) {
          return true;
        }
        ec_send_processdata();
        return true;
    }

    /*! Do a sanity check of all devices on this bus.
     */
    void sanityCheck() {
        if(inOP_ && ((wkc_ < wkcExpected_) || ec_group[currentgroup_].docheckstate))
        {
            if (needlf_)
            {
                needlf_ = false;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup_].docheckstate = FALSE;
            ec_readstate();
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup_) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup_].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, timeoutmon_))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n",slave);
                        }
                    }
                    else if(!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n",slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if(!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, timeoutmon_))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n",slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n",slave);
                    }
                }
            }
            if(!ec_group[currentgroup_].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }






        bool isMissingOrError = false;
        bool allActive = true;
        for(auto device : devices_) {
            device->sanityCheck();
            isMissingOrError |= device->isMissing() | device->hasError();
            allActive &= device->isActive();
        }

        isMissingDeviceOrHasError_ = isMissingOrError;
        allDevicesActive_ = allActive;
    }
protected:
    inline std::shared_ptr<Datagram> addDatagram(Datagram&& datagram) {
        // put the datagram at apropriate position to replicate the physical structure of the bus in the datagram order?
        auto ptr = std::make_shared<Datagram>(std::forward<Datagram>(datagram));
        datagrams_.push_back(ptr);
        return ptr;
    }

    bool allDevicesFound() {
        // Verify the expected number of slaves
        if (static_cast<size_t>(ec_slavecount) < devices_.size()) {
            return false;
        }

        // Verify slave by slave that the name is correct
        for (const EtherCatDevice* device : devices_) {
            if (std::string(ec_slave[device->getNodeId()].name) != device->getName()) {
                return false;
            }
        }

        return true;
    }

protected:
    // vector containing all devices
    std::vector<EtherCatDevice*> devices_;

    // map mapping COB id to parse functions
    AddressToFunctionMap addressToFunctionMap_;

    std::vector<std::shared_ptr<Datagram>> datagrams_;

    const int timeoutmon_ = 500;
    char IOmap_[4096];
    std::atomic<int> wkcExpected_;
    std::atomic<int> wkc_;
    std::atomic<bool> needlf_;
    std::atomic<bool> inOP_;
    uint8 currentgroup_ = 0;
};

} /* namespace tcan */

