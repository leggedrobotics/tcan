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
        std::copy(&oldData[0], &oldData[header_.lenRCM_.lenRCMElements_.len_], data_);
        header_.lenRCM_.lenRCMElements_.len_ = length;
        delete[] oldData;
    }

    template <typename T>
    inline void write(const uint16_t memoryPosition, const T& data) {
        // check if memoryPosition lies within data_?
        std::copy(&data_[memoryPosition], &data_[memoryPosition + sizeof(T)], &data);
    }

    inline const uint16_t getTotalLength() const { return getDataLength() + 12; }
    inline const uint16_t getDataLength() const { return header_.lenRCM_.lenRCMElements_.len_; }
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
        // Note: Bit field ordering depends on the Endianness which is implementation dependent.
        struct LenRCMBits {
          uint16_t len_           :11; // does this sub-byte work?
          uint16_t reserved_      :3;
          uint16_t circulating_   :1;
          uint16_t more_          :1;
        };
        // Note: The constructor per default calls the initializer of the first element of a union.
        union LenRCM {
          uint16_t lenRCMAll_;
          LenRCMBits lenRCMElements_;
        };

        Command cmd_;
        uint8_t idx_;
        uint32_t address_;
        LenRCM lenRCM_;
        uint16_t irq_;

//        DatagramHeader(): cmd_(Command::NOP), idx_(0), address_(0), len_(0), reserved_(0), circulating_(0), more_(0), irq_(0) { }
        DatagramHeader(): cmd_(Command::NOP), idx_(0), address_(0), lenRCM_{0}, irq_(0) { }

//        uint16_t getLength() const {
//            return (lenRCM_ & 0xFFE0) >> 5;
//        }
//
//        void setLength(uint16_t length) {
//            lenRCM_ &= ~0xFFE0; // clear bits first.
//            lenRCM_ = (length << 5) & 0xFFE0;
//        }
//
//        uint16_t getReserved() const {
//            return (lenRCM_ & 0x001C) >> 2;
//        }
//
//        void setReserved(uint16_t reserved) {
//            lenRCM_ &= ~0x001C; // clear bits first.
//            lenRCM_ = (reserved << 2) & 0x001C;
//        }
//
//        uint16_t getCirculating() const {
//            return (lenRCM_ & 0x0002) >> 1;
//        }
//
//        void setCirculating(uint16_t circulating) {
//            lenRCM_ &= ~0x0002; // clear bits first.
//            lenRCM_ = (circulating << 1) & 0x0002;
//        }
//
//        uint16_t getMore() const {
//            return (lenRCM_ & 0x0001) >> 0;
//        }
//
//        void setMore(uint16_t more) {
//            lenRCM_ &= ~0x0001; // clear bits first.
//            lenRCM_ = (more << 1) & 0x0001;
//        }
    } __attribute__((packed)); // prevent structure padding


 public:
    DatagramHeader header_;
    uint8_t* data_ = nullptr;
    uint16_t workingCounter_ = 0;

    // have a map of (name string => memory address) to have human-readable access to the fields in data_? would need to update this on resize(..) calls..
};




typedef enum ECAT_DSP402_COMMAND_TYPE
{
    SWITCH_ON           = 0x00,
    SHUTDOWN            ,
    DISABLE_VOLTAGE     ,
    ENABLE_VOLTAGE      ,
    QUICK_STOP          ,
    DISABLE_OPERATION   ,
    ENABLE_OPERATION    ,
    FAULT_RESET         ,
    HALT                ,
    HALT_RESET          ,
    CLEAR_CONTROLWORD   ,

} dsp402_command_e;

// Note: Bit field ordering depends on the Endianness which is implementation dependent.
typedef struct DSP402_STATUSWORD_BITS
{
    uint16_t ready_to_switch_on:1;
    uint16_t switched_on:1;
    uint16_t operation_enabled:1;
    uint16_t fault:1;
    uint16_t voltage_enabled:1;
    uint16_t quick_stop:1;
    uint16_t switch_on_disabled:1;
    uint16_t warning:1;
    uint16_t manufacturer_specific_0:1;
    uint16_t remote:1;
    uint16_t operation_mode_specific_0:1;
    uint16_t internal_limit_active:1;
    uint16_t operation_mode_specific_1:1;
    uint16_t operation_mode_specific_2:1;
    uint16_t manufacturer_specific_1:2;

} dsp402_statusword_bits_t;

typedef union DSP402_STATUSWORD_TYPE
{
    uint16_t all;
    dsp402_statusword_bits_t bits;

} dsp402_statusword_t;

typedef struct DSP402_CONTROLWORD_BITS
{
    uint16_t switch_on:1;
    uint16_t enable_voltage:1;
    uint16_t quick_stop:1;
    uint16_t enable_operation:1;
    uint16_t operation_mode_specific_0:3;
    uint16_t fault_reset:1;
    uint16_t halt:1;
    uint16_t operation_mode_specific_1:1;
    uint16_t rsrvd:1;
    uint16_t manufacturer_specific:5;

} dsp402_controlword_bits_t;

typedef union DSP402_CONTROLWORD_TYPE
{
    uint16_t all;
    dsp402_controlword_bits_t bits;

} dsp402_controlword_t;

typedef struct ELMO_INDATA_TYPE
{
    dsp402_statusword_t statusword;

    int position;
    int velocity;

    int digitalin;

    int busvoltage;
    int motorcurrent;

} elmo_twitter_indata_t;

typedef struct ELMO_OUTDATA_TYPE
{
    dsp402_controlword_t controlword;

    int torque;

} elmo_twitter_outdata_t;



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
      inOP_(false),
      ecatContext_({
          &ecat_port,          // .port          =
          &ecat_slave[0],       // .slavelist     =
          &ecat_slavecount,     // .slavecount    =
          EC_MAXSLAVE,        // .maxslave      =
          &ecat_group[0],       // .grouplist     =
          EC_MAXGROUP,        // .maxgroup      =
          &ecat_esibuf[0],      // .esibuf        =
          &ecat_esimap[0],      // .esimap        =
          0,                  // .esislave      =
          &ecat_elist,          // .elist         =
          &ecat_idxstack,       // .idxstack      =
          &ecat_error,         // .ecaterror     =
          0,                  // .DCtO          =
          0,                  // .DCl           =
          &ecat_DCtime,         // .DCtime        =
          &ecat_SMcommtype[0],  // .SMcommtype    =
          &ecat_PDOassign[0],   // .PDOassign     =
          &ecat_PDOdesc[0],     // .PDOdesc       =
          &ecat_SM,             // .eepSM         =
          &ecat_FMMU,           // .eepFMMU       =
          NULL                // .FOEhook()
      }) {}

    virtual ~EtherCatBus() {
        exit();
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
            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.lenRCM_.lenRCMAll_ >> 0) & 0xFF);
            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.lenRCM_.lenRCMAll_ >> 8) & 0xFF);
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

        static int test_step=0;
        static int step_counter=0;

        // Test procedure
        switch (test_step)
        {
            case 0: // Reset errors
                if (step_counter >= 2000)
                {
                    step_counter = 0;
                    test_step++;
                }
                else if (step_counter == 1)
                {
                    ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, FAULT_RESET, 0.0);
                    ecatcomm_slave_print_controlword(outdata_.controlword);
                    ecatcomm_slave_print_statusword(indata_.statusword);
                    printf("Clearing errors...\n\n");
                }
                break;

            case 1: // Startup
                if (step_counter >= 5000)
                {
                    step_counter = 0;
                    test_step++;
                }
                else if (step_counter == 1)
                {
                    ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
                    ecatcomm_slave_print_controlword(outdata_.controlword);
                    ecatcomm_slave_print_statusword(indata_.statusword);
                    printf("Startup up drive...\n\n");
                }
                break;

            case 2: // Switch on
                if (step_counter >= 5000)
                {
                    step_counter = 0;
                    test_step++;
                }
                else if (step_counter == 1)
                {
                    ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, SWITCH_ON, 0.0);
                    ecatcomm_slave_print_controlword(outdata_.controlword);
                    ecatcomm_slave_print_statusword(indata_.statusword);
                    printf("Switching on drive...\n\n");
                }
                break;
            case 3: // enable operation
                if(step_counter >= 5000)
                {
                    step_counter = 0;
                    test_step++;
                }
                else if (step_counter == 1)
                {
                    ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, SWITCH_ON, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_OPERATION, 0.0);
                    ecatcomm_slave_print_controlword(outdata_.controlword);
                    ecatcomm_slave_print_statusword(indata_.statusword);
                    printf("Enabling operation...\n\n");
                }
            break;
            case 4: // wait delay
                if(step_counter >= 7000)
                {
                    step_counter = 0;
                    test_step++;
                }
                else if (step_counter == 1)
                {
                    ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, SWITCH_ON, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_OPERATION, 0.0);
                    ecatcomm_slave_print_controlword(outdata_.controlword);
                    ecatcomm_slave_print_statusword(indata_.statusword);
                    printf("Running test...\n\n");
                }
                break;
            case 5: // Run test output
                if(step_counter >= 7000)
                {
                    step_counter = 0;
                    test_step++;
                }
                else if (step_counter == 1)
                {
                    ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, SWITCH_ON, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_OPERATION, 0.0);
                    ecatcomm_slave_print_controlword(outdata_.controlword);
                    ecatcomm_slave_print_statusword(indata_.statusword);
                    printf("Running test...\n\n");
                }
                break;
            case 6: // Stop before end
                if(step_counter >= 5000)
                {
                    step_counter = 0;
                    test_step++;
                }
                else if (step_counter == 1)
                {
                    ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, SWITCH_ON, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_OPERATION, 0.0);
                    ecatcomm_slave_print_controlword(outdata_.controlword);
                    ecatcomm_slave_print_statusword(indata_.statusword);
                    printf("Stopping test...\n\n");
                }
                break;
            case 7: // Stop before end
                if(step_counter >= 5000)
                {
                    step_counter = 0;
                    test_step++;
                    running_ = false;
                }
                else if (step_counter == 1)
                {
                    ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, SWITCH_ON, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
                    ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
                    ecatcomm_slave_print_controlword(outdata_.controlword);
                    ecatcomm_slave_print_statusword(indata_.statusword);
                    printf("Exiting test...\n\n");
                }
                break;
        }

        step_counter++;
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
        if (ecx_init(&ecatContext_, ifname) <= 0) {
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
        if (ecx_config_init(&ecatContext_, FALSE) == 0) {
            MELO_ERROR_STREAM("No slaves have been found.");
            return false;
        }

        MELO_INFO_STREAM("The following " << *ecatContext_.slavecount << " slaves have been found and configured:");
        for (int i = 1; i <= *ecatContext_.slavecount; i++) {
            MELO_INFO_STREAM(i << ": " << std::string(ecatContext_.slavelist[i].name));
        }

        if (!allDeviceMatch()) {
            MELO_ERROR_STREAM("Expected and discovered devices mismatch.");
            return false;
        }
        MELO_INFO_STREAM("Expected and discovered devices match.");

        /*!
         * We now have the network up and configured. Mailboxes are up for slaves that
         * support it. Next we will create an IOmap and configure the SyncManager's and
         * FMMU's to link the EtherCAT master and the slaves. The IO mapping is done
         * automatically, SOEM strives to keep the logical process image as compact as
         * possible. It is done by trying to fit Bit oriented slaves together in single
         * bytes. Below is an example of 8 slaves and how they are ordered. During
         * mapping SOEM also calculates an expected wkc_ for the IO mapped together.
         * That is the primary key to detect errors.
         *
         *  * Outputs are placed together in the beginning of the IOmap
         *  * Inputs follow
         *
         * When the mapping is done SOEM requests slaves to enter SAFE_OP.
         */
        ecx_config_map_group(&ecatContext_, &IOmap_, 0);

        ecx_configdc(&ecatContext_); // TODO what is this doing?

        printf("Slaves mapped, state to SAFE_OP.\n");
        /* wait for all slaves to reach SAFE_OP state */
        ecx_statecheck(&ecatContext_, 0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

        int oloop_ = ecatContext_.slavelist[0].Obytes;
        if ((oloop_ == 0) && (ecatContext_.slavelist[0].Obits > 0)) oloop_ = 1;
//        if (oloop_ > 8) oloop_ = 8;
        int iloop_ = ecatContext_.slavelist[0].Ibytes;
        if ((iloop_ == 0) && (ecatContext_.slavelist[0].Ibits > 0)) iloop_ = 1;
//        if (iloop_ > 8) iloop_ = 8;

        printf("segments : %d : %d %d %d %d\n",ecatContext_.grouplist[0].nsegments, ecatContext_.grouplist[0].IOsegment[0], ecatContext_.grouplist[0].IOsegment[1], ecatContext_.grouplist[0].IOsegment[2], ecatContext_.grouplist[0].IOsegment[3]);





        checkSlaveStates();
        configureSlave();
        checkSlaveStates();




        if (ecatContext_.slavelist[0].state == EC_STATE_OPERATIONAL ) {
            printf("Operational state reached for all slaves.\n");
            inOP_ = true;

            // Start preparation
            running_ = true;

            // Handle PDO streams
            outdata_.controlword.all = 0;
            ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
            ecatcomm_slave_set_rxpdo(&outdata_, SHUTDOWN, 0.0);
            ecx_send_processdata(&ecatContext_);
            wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
            ecatcomm_slave_get_txpdo(&indata_);
            osal_usleep(1000);
        }
        else {
            printf("Not all slaves reached operational state.\n");
            ecx_readstate(&ecatContext_);
            for(int i = 1; i <= *ecatContext_.slavecount; i++) {
                if(ecatContext_.slavelist[i].state != EC_STATE_OPERATIONAL) {
                    printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                    i, ecatContext_.slavelist[i].state, ecatContext_.slavelist[i].ALstatuscode, ec_ALstatuscode2string(ecatContext_.slavelist[i].ALstatuscode));
                }
            }
        }





//        printf("Request operational state for all slaves\n");
//        wkcExpected_ = (ec_group[0].outputswkc_ * 2) + ec_group[0].inputsWKC;
//        printf("Calculated workcounter %d\n", wkcExpected_.load());
//        ec_slave[0].state = EC_STATE_OPERATIONAL;
//        /* send one valid process data to make outputs in slaves happy*/
//        ec_send_processdata();
//        ec_receive_processdata(EC_TIMEOUTRET);
//        /* request OP state for all slaves */
//        ec_writestate(0);
//        int chk = 40;
//        /* wait for all slaves to reach OP state */
//        do
//        {
//            ec_send_processdata();
//            ec_receive_processdata(EC_TIMEOUTRET);
//            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
//        }
//        while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
//        if (ec_slave[0].state == EC_STATE_OPERATIONAL )
//        {
//            printf("Operational state reached for all slaves.\n");
//            inOP_ = true;
//            /* cyclic loop */
//            for(int i = 1; i <= 10000; i++)
//            {
//                ec_send_processdata();
//                wkc_ = ec_receive_processdata(EC_TIMEOUTRET);
//
//                if(wkc_ >= wkcExpected_)
//                {
//                    printf("Processdata cycle %4d, wkc_ %d , O:", i, wkc_.load());
//
//                    for(int j = 0 ; j < oloop; j++)
//                    {
//                        printf(" %2.2x", *(ec_slave[0].outputs + j));
//                    }
//
//                    printf(" I:");
//                    for(int j = 0 ; j < iloop; j++)
//                    {
//                        printf(" %2.2x", *(ec_slave[0].inputs + j));
//                    }
//                    printf(" T:%" PRId64 "\r",ec_DCtime);
//                    needlf_ = true;
//                }
//                osal_usleep(5000);
//
//            }
//            inOP_ = false;
//        }
//        else
//        {
//            printf("Not all slaves reached operational state.\n");
//            ec_readstate();
//            for(int i = 1; i<=ec_slavecount ; i++)
//            {
//                if(ec_slave[i].state != EC_STATE_OPERATIONAL)
//                {
//                    printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
//                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
//                }
//            }
//        }
//        printf("\nRequest init state for all slaves\n");
//        ec_slave[0].state = EC_STATE_INIT;
//        /* request INIT state for all slaves */
//        ec_writestate(0);


        return true;
    }

    /*! read CAN message from the device driver
     * @return true if a message was successfully read and parsed
     */
    virtual bool readData() {
//        if (!inOP_) {
//          MELO_WARN_STREAM("Devices are not in OP.");
//          return false;
//        }
//        wkc_ = ec_receive_processdata(EC_TIMEOUTRET);
//        MELO_INFO_STREAM("Working counter is at " << wkc_ << "/" << wkcExpected_ << ".");
//        if (wkc_ < wkcExpected_) {
//          MELO_WARN_STREAM("Working counter is too low.");
//          return false;
//        }
//        EthernetFrame ethernetFrame;
//        handleMessage(ethernetFrame);
        return false;
    }

    /*! write CAN message to the device driver
     * @return true if the message was successfully written
     */
    virtual bool writeData(const EthernetFrame& msg) {
        static int i = 0;
        static int print_counter=0;

        if (!running_) {
            std::cout << "is not running" << std::endl;
            return false;
        }
//        std::cout << "writing" << std::endl;

        // Step
        i++;

        // Handle PDO streams
        ecx_send_processdata(&ecatContext_);
        wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
        ecatcomm_slave_get_txpdo(&indata_);

        if((wkc_ >= wkcExpected_) && (++print_counter>=5))
        {
            printf("Processdata cycle %4d, WKC %d", i, wkc_.load());

            printf(", Outputs:");
            for(int j = 0 ; j < oloop_; j++)
            {
                printf(" %2.2x", *(ecatContext_.slavelist[0].outputs + j));
            }
            // printf(", Inputs:");
            // for(j = 0 ; j < iloop; j++)
            // {
            //     printf(" %2.2x", *(ecatContext_.slavelist[0].inputs + j));
            // }

            // printf(" T:%"PRId64"",ecatContext_.DCtime[0]);
            printf(", Command Data: 0x%4x, %4d", outdata_.controlword.all, outdata_.torque);
            printf(", Feedback Data: 0x%4x, %8d, %8d, %8d, %8d", indata_.statusword.all, indata_.position, indata_.velocity, indata_.busvoltage, indata_.motorcurrent);
            printf("\r");
            print_counter = 0;
        }

//        inOP_ = false;

        return true;
    }

    /*! Do a sanity check of all devices on this bus.
     */
    void sanityCheck() {
        if(inOP_ && ((wkc_ < wkcExpected_) || ecatContext_.grouplist[currentgroup_].docheckstate)) {
            /* one ore more slaves are not responding */
            ecatContext_.grouplist[currentgroup_].docheckstate = FALSE;
            ecx_readstate(&ecatContext_);
            for (int slave = 1; slave <= *ecatContext_.slavecount; slave++) {
                if ((ecatContext_.slavelist[slave].group == currentgroup_) && (ecatContext_.slavelist[slave].state != EC_STATE_OPERATIONAL))
                {
                    ecatContext_.grouplist[currentgroup_].docheckstate = TRUE;
                    if (ecatContext_.slavelist[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ecatContext_.slavelist[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ecx_writestate(&ecatContext_, slave);
                    }
                    else if(ecatContext_.slavelist[slave].state == EC_STATE_SAFE_OP) {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ecatContext_.slavelist[slave].state = EC_STATE_OPERATIONAL;
                        ecx_writestate(&ecatContext_, slave);
                    }
                    else if(ecatContext_.slavelist[slave].state > 0) {
                        if (ecx_reconfig_slave(&ecatContext_, slave, timeoutmon_)) {
                            ecatContext_.slavelist[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n",slave);
                        }
                    }
                    else if(!ecatContext_.slavelist[slave].islost) {
                        /* re-check state */
                        ecx_statecheck(&ecatContext_, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ecatContext_.slavelist[slave].state) {
                            ecatContext_.slavelist[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n",slave);
                        }
                    }
                }
                if (ecatContext_.slavelist[slave].islost) {
                    if(!ecatContext_.slavelist[slave].state) {
                        if (ecx_recover_slave(&ecatContext_, slave, timeoutmon_)) {
                          ecatContext_.slavelist[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n",slave);
                        }
                    }
                    else {
                        ecatContext_.slavelist[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n",slave);
                    }
                }
            }
            if(!ecatContext_.grouplist[currentgroup_].docheckstate) {
                printf("OK : all slaves resumed OPERATIONAL.\n");
            }
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

    bool allDeviceMatch() {
        // Verify the expected number of slaves
        if (*ecatContext_.slavecount < static_cast<int>(devices_.size())) {
            MELO_ERROR_STREAM("Found too few slaves (" << *ecatContext_.slavecount << " < " << static_cast<int>(devices_.size()) << ").")
            return false;
        }
        else if (*ecatContext_.slavecount > static_cast<int>(devices_.size())) {
          MELO_ERROR_STREAM("Found too many slaves (" << *ecatContext_.slavecount << " < " << static_cast<int>(devices_.size()) << ").")
            return false;
        }

        // Verify slave by slave that the name is correct
        for (const EtherCatDevice* device : devices_) {
            if (std::string(ecatContext_.slavelist[device->getAddress()].name) != device->getName()) {
                MELO_ERROR_STREAM("Device with address '" << device->getAddress() << "' expected name '" << device->getName() << "' but found '" << std::string(ecatContext_.slavelist[device->getAddress()].name) << "'.");
                return false;
            }
        }

        return true;
    }

    void getSlaveInfo() {
        int ret = 0;
        int i = 0, k = 0, j = 0, n = 0;
        int wkc_ = 0;
        int32_t actual_position = 0;
        int posdatasize = sizeof(int32_t);
        int Osize = 0, Isize = 0;

        #define Nsdo  16
        #define Ndata 64
        int sdodata[Nsdo], sdodatasize;
        char *databuf;
        databuf = (char*)&sdodata[0];

        // Check PDO mapping size
        ret = ecx_readPDOmap(&ecatContext_, 1, &Osize, &Isize);
        printf("\n\nec_readPDOmap returned %d\n", ret);
        printf("Osize in bits = %d\n", Osize);
        printf("Isize in bits = %d\n", Isize);

        // Get complete OD dump
        ret = ecx_readODlist(&ecatContext_, 1, &odinfo_);
        printf("\nc_readODlist returned %d\n", ret);
        printf("Slave = %d, Entries = %d\n", odinfo_.Slave, odinfo_.Entries);
        for (k = 0; k < odinfo_.Entries; k++) {
            wkc_ = ecx_readODdescription(&ecatContext_, k, &odinfo_);
            wkc_ = ecx_readOE(&ecatContext_, k, &odinfo_, &odentryinfo_);
            printf("\nIndex = 0x%x\n", odinfo_.Index[k]);
            printf("    MaxSub     = %d\n", odinfo_.MaxSub[k]+1);
            printf("    ObjectCode = %d\n", odinfo_.ObjectCode[k]);
            printf("    DataType   = %d\n", odinfo_.DataType[k]);
            printf("    Description: %s\n", &odinfo_.Name[k][0]);
            printf("    OE Entries = %d\n", odentryinfo_.Entries);
            for (j = 0; j < odentryinfo_.Entries; j++) {
                for (n=0; n<Nsdo; n++) sdodata[n] = 0;
                sdodatasize = Nsdo*sizeof(int);
                wkc_ = ecx_SDOread(&ecatContext_, odinfo_.Slave, odinfo_.Index[k], j, 0, &sdodatasize, &sdodata, EC_TIMEOUTRXM);
                printf("    OE = %d\n", j);
                printf("        ValueInfo  = %d\n", odentryinfo_.ValueInfo[j]);
                printf("        DataType   = %d\n", odentryinfo_.DataType[j]);
                printf("        BitLength  = %d\n", odentryinfo_.BitLength[j]);
                printf("        ObjAccess  = %d\n", odentryinfo_.ObjAccess[j]);
                printf("        Name       = %s\n", &odentryinfo_.Name[j][0]);
                printf("        Value      =");
                for (n=0; n<sdodatasize; n++) printf(" 0x%x", (0xFF & databuf[n]));
                printf("\n");
            }
        }
    }

    void configureSlave() {
        int i=0;
        uint8 bufferu8 = 0;
        uint16 bufferu16 = 0;
        uint32 bufferu32 = 0;
        uint64 bufferu64 = 0;
        int8 buffers8 = 0;
        int16 buffers16 = 0;
        int32 buffers32 = 0;
        int64 buffers64 = 0;

//        dsp402_controlword_t controlword = {0};
//        elmo_twitter_indata_t indata;
//        elmo_twitter_outdata_t outdata;

        // Set state
        ecatContext_.slavelist[0].state = EC_STATE_PRE_OP;
        ecx_writestate(&ecatContext_, 0);
        ecx_statecheck(&ecatContext_, 0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE * 4);

        printf("\nSetting device configuratons...\n");

        // RxPDO assignments in SM2
        bufferu8 = 1;
        wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x1c12, 0, FALSE, 1, &bufferu8, EC_TIMEOUTRXM);
        ecatcomm_slave_check_sdo(0x1c12, 0, FALSE);

        bufferu16 = 0x1602;
        wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x1c12, 1, TRUE, 2, &bufferu16, EC_TIMEOUTRXM);
        ecatcomm_slave_check_sdo(0x1c12, 1, TRUE);

        // RxPDO assignments in SM3
        bufferu8 = 3;
        wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x1c13, 0, FALSE, 1, &bufferu8, EC_TIMEOUTRXM);
        ecatcomm_slave_check_sdo(0x1c13, 0, FALSE);

        bufferu64 = 0x1a1f1a181a03;
        wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x1c13, 1, TRUE, 8, &bufferu64, EC_TIMEOUTRXM);
        // ecatcomm_slave_check_sdo(0x1c13, 1, TRUE);
        ecatcomm_slave_check_sdo(0x1c13, 1, FALSE);
        ecatcomm_slave_check_sdo(0x1c13, 2, FALSE);
        ecatcomm_slave_check_sdo(0x1c13, 3, FALSE);

        // bufferu16 = 0x1a03; // position + velocity feedback
        // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x1c13, 1, TRUE, 2, &bufferu16, EC_TIMEOUTRXM);
        // ecatcomm_slave_check_sdo(0x1c13, 1, TRUE);
        //
        // bufferu16 = 0x1a18; // DC bus voltage
        // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x1c13, 2, TRUE, 2, &bufferu16, EC_TIMEOUTRXM);
        // ecatcomm_slave_check_sdo(0x1c13, 2, TRUE);
        //
        // bufferu16 = 0x1a1f; // motor current
        // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x1c13, 3, TRUE, 2, &bufferu16, EC_TIMEOUTRXM);
        // ecatcomm_slave_check_sdo(0x1c13, 3, TRUE);

        // DC Sync0
        double time_step = 1e-3;
        ecx_dcsync0(&ecatContext_, 1, TRUE, (int64)(time_step*1e9), (int64)(time_step*0.5*1e9));

        // // enable voltage + quick-stop + fault reset
        // controlword.bits.quick_stop = 1;
        // controlword.bits.enable_voltage = 1;
        // controlword.bits.fault_reset = 1;
        // bufferu16 = controlword.all;
        // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x6040, 0, FALSE, 2, &bufferu16, EC_TIMEOUTRXM);
        // ecatcomm_slave_check_sdo(0x6040, 0, FALSE);
        // ecatcomm_slave_check_sdo(0x6041, 0, FALSE);

        // Set state
        ecatContext_.slavelist[0].state = EC_STATE_SAFE_OP;
        ecx_writestate(&ecatContext_, 0);
        ecx_statecheck(&ecatContext_, 0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

        // // Unkown config
        // buffers16 = 0x5f;
        // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x3034, 7, FALSE, 2, &buffers16, EC_TIMEOUTRXM);
        // ecatcomm_slave_check_sdo(0x3034, 7, FALSE);
        //
        // // Halp Option Code
        // bufferu8 = 3;
        // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x605D, 0, TRUE, 1, &bufferu8, EC_TIMEOUTRXM);
        // ecatcomm_slave_check_sdo(0x605D, 0, TRUE);

        // buffers8 = 3;
        // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x605D, 0, FALSE, 1, &buffers8, EC_TIMEOUTRXM);
        // ecatcomm_slave_check_sdo(0x605D, 0, FALSE);

        bufferu8 = 4;
        wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x6060, 0, FALSE, 1, &bufferu8, EC_TIMEOUTRXM);
        ecatcomm_slave_check_sdo(0x6060, 0, FALSE);
        ecatcomm_slave_check_sdo(0x6061, 0, FALSE);

        // Set initial process data - SWITCH ON
        // outdata.controlword.all = 0;
        // outdata.controlword.bits.enable_voltage = 1;
        // outdata.controlword.bits.quick_stop = 1;
        // outdata.controlword.bits.fault_reset = 1;
        // for (i=0; i<1000; i++)
        // {
        //     ecatcomm_slave_set_rxpdo(&outdata, SWITCH_ON, 0.0);
        //     ecx_send_processdata(&ecatContext_);
        //     ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
        //     ecatcomm_slave_get_txpdo(&indata);
        //     ecatcomm_slave_print_controlword(controlword);
        //     ecatcomm_slave_print_statusword(indata.statusword);
        //     osal_usleep(1000);
        // }

        // Set state to Ethercat Operatoinal
        ecatContext_.slavelist[0].state = EC_STATE_OPERATIONAL;
        ecx_writestate(&ecatContext_, 0);
        ecx_statecheck(&ecatContext_, 0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE * 4);

        // send one valid process data to make outputs in slaves happy
        ecx_send_processdata(&ecatContext_);
        wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);

        // wait for all slaves to reach OP state
        int chk = 40;
        do {
            ecx_send_processdata(&ecatContext_);
            wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
            ecx_statecheck(&ecatContext_, 0, EC_STATE_OPERATIONAL,  50000);
        } while (chk-- && (ecatContext_.slavelist[0].state != EC_STATE_OPERATIONAL));

        // Get error data
        dumpSlaveStatusInfo();
    }

    void dumpSlaveStatusInfo() {
        printf("\nDevice Errors: \n");
        ecatcomm_slave_check_sdo(0x1001, 0, FALSE);

        ecatcomm_slave_check_sdo(0x1002, 0, FALSE);

        ecatcomm_slave_check_sdo(0x1003, 0, TRUE);

        ecatcomm_slave_check_sdo(0x2081, 0, FALSE);
        ecatcomm_slave_check_sdo(0x2081, 1, FALSE);
        ecatcomm_slave_check_sdo(0x2081, 2, FALSE);
        ecatcomm_slave_check_sdo(0x2081, 3, FALSE);
        ecatcomm_slave_check_sdo(0x2081, 4, FALSE);
        ecatcomm_slave_check_sdo(0x2081, 5, FALSE);
        ecatcomm_slave_check_sdo(0x2081, 6, FALSE);

        ecatcomm_slave_check_sdo(0x2085, 0, FALSE);
    }

    void checkSlaveStates() {
        int slavestate=0, wkc=0;
        int sdodata=0, sdodatasize=sizeof(int);

        slavestate = ecx_readstate(&ecatContext_);
        printf("\nSlave EtherCAT StateMachine state is 0x%x\n", slavestate);

        sdodata = 0;
        sdodatasize = sizeof(int);
        wkc_ = ecx_SDOread(&ecatContext_, 1, 0x6041, 0, 0, &sdodatasize, &sdodata, EC_TIMEOUTRXM);
        printf("Slave DSP402 StateMachine state is 0x%x\n", (uint16)sdodata);
    }

    void checkSlaveErrors() {
        int i=0, Nt=0, Nh=0, Nerr=0;
        ec_errort error;
        char *errstr;

        printf("\nERROR REPORT:\n");

        // Built-in error reporting
        if (ecx_iserror(&ecatContext_)) {
            do {
                errstr = ecx_elist2string(&ecatContext_);
                printf("    Error %d: %s", ++i, errstr);
            } while(ecx_iserror(&ecatContext_));
        }
    }

    void exit() {
        /* request INIT state for all slaves */
        printf("\nRequest init state for all slaves\n");
        ecatContext_.slavelist[0].state = EC_STATE_INIT;
        ecx_writestate(&ecatContext_, 0);

        /* stop SOEM, close socket */
        printf("End simple test, close socket\n");
        ecx_close(&ecatContext_);
        printf("Closed socket\n");
    }

    void ecatcomm_slave_check_sdo(int index, int subindex, boolean CA)
    {
        int sdodata[4]={0,0}, sdodatasize=16;
        wkc_ = ecx_SDOread(&ecatContext_, 1, index, subindex, CA, &sdodatasize, &sdodata, EC_TIMEOUTRXM);
        printf(" OD entry {Index 0x%x, Subindex 0x%x} is  0x%x 0x%x 0x%x 0x%x\n", index, subindex, sdodata[3], sdodata[2], sdodata[1], sdodata[0]);
    }

    void ecatcomm_slave_set_rxpdo(elmo_twitter_outdata_t *pdata, int command, double torque)
    {
        static dsp402_controlword_t controlword = {0};
        int16_t torque_data=0;
        char databuffer[4];

        // Set controlword data
        switch(command)
        {
            case SWITCH_ON:
                controlword.bits.switch_on = 1;
                break;

            case SHUTDOWN:
                controlword.bits.switch_on = 0;
                break;

            case DISABLE_VOLTAGE:
                controlword.bits.enable_voltage = 0;
                break;

            case ENABLE_VOLTAGE:
                controlword.bits.enable_voltage = 1;
                break;

            case QUICK_STOP:
                controlword.bits.quick_stop = 1;
                break;

            case DISABLE_OPERATION:
                controlword.bits.enable_operation = 0;
                break;

            case ENABLE_OPERATION:
                controlword.bits.enable_operation = 1;
                break;

            case FAULT_RESET:
                controlword.bits.fault_reset = 1;
                break;

            case HALT:
                controlword.bits.halt = 1;
                break;

            case HALT_RESET:
                controlword.bits.halt = 0;
                break;

            case CLEAR_CONTROLWORD:
                controlword.all = 0;
                return;
        }

        // Covert torque data to INT16 from double
        double rated_current = 20000.0;
        double rated_torque = (20000.0*0.27)*0.001;
        torque_data = (int16_t)(torque/rated_torque*1000.0);

        // Copy to internal struct
        pdata->controlword.all = controlword.all;
        pdata->torque = torque_data;

        // Write to output buffer
        databuffer[0] = ((controlword.all >> 0) & 0xff);
        databuffer[1] = ((controlword.all >> 8) & 0xff);
        databuffer[2] = ((torque_data >> 0) & 0xff);
        databuffer[3] = ((torque_data >> 8) & 0xff);

        // Set data into buffer
        memcpy(ecatContext_.slavelist[1].outputs, &databuffer[0], 4);
    }

    void ecatcomm_slave_get_txpdo(elmo_twitter_indata_t *pdata)
    {
        uint16 statusword=0;
        char databuffer[21];

        // Get data
        memcpy(&databuffer[0], ecatContext_.slavelist[1].inputs, 21);

        // Store data
        pdata->statusword.all = ((databuffer[13] << 8 ) & 0xff00) | (databuffer[12] & 0xff);
        pdata->position = ((databuffer[3] << 24) & 0xff000000) | ((databuffer[2] << 16) & 0x00ff0000) | ((databuffer[1] << 8) & 0x0000ff00) | ((databuffer[0] << 0) & 0x000000ff);
        pdata->digitalin = ((databuffer[7] << 24) & 0xff000000) | ((databuffer[6] << 16) & 0x00ff0000) | ((databuffer[5] << 8) & 0x0000ff00) | ((databuffer[4] << 0) & 0x000000ff);
        pdata->velocity = ((databuffer[11] << 24) & 0xff000000) | ((databuffer[10] << 16) & 0x00ff0000) | ((databuffer[9] << 8) & 0x0000ff00) | ((databuffer[8] << 0) & 0x000000ff);
        pdata->busvoltage = ((databuffer[18] << 24) & 0xff000000) | ((databuffer[17] << 16) & 0x00ff0000) | ((databuffer[16] << 8) & 0x0000ff00) | ((databuffer[15] << 0) & 0x000000ff);
        pdata->motorcurrent = ((databuffer[20] << 8) & 0xff00) | ((databuffer[19] << 0) & 0x00ff);
    }

    void ecatcomm_slave_print_statusword(dsp402_statusword_t statusword)
    {
        // Printout
        printf("\n\n");
        printf("statusword.ready_to_switch_on         = %d\n", statusword.bits.ready_to_switch_on);
        printf("statusword.switched_on                = %d\n", statusword.bits.switched_on);
        printf("statusword.operation_enabled          = %d\n", statusword.bits.operation_enabled);
        printf("statusword.fault                      = %d\n", statusword.bits.fault);
        printf("statusword.volt_enabled               = %d\n", statusword.bits.voltage_enabled);
        printf("statusword.quick_stop                 = %d\n", statusword.bits.quick_stop);
        printf("statusword.switch_on_disabled         = %d\n", statusword.bits.switch_on_disabled);
        printf("statusword.warning                    = %d\n", statusword.bits.warning);
        printf("statusword.manufacturer_specific      = %d\n", statusword.bits.manufacturer_specific_0);
        printf("statusword.remote                     = %d\n", statusword.bits.remote);
        printf("statusword.operation_mode_specific_0  = %d\n", statusword.bits.operation_mode_specific_0);
        printf("statusword.internal_limit_active      = %d\n", statusword.bits.internal_limit_active);
        printf("statusword.operation_mode_specific_1  = %d\n", statusword.bits.operation_mode_specific_1);
        printf("statusword.operation_mode_specific_2  = %d\n", statusword.bits.operation_mode_specific_2);
        printf("statusword.manufacturer_specific      = %d\n", statusword.bits.manufacturer_specific_1);
    }

    void ecatcomm_slave_print_controlword(dsp402_controlword_t controlword)
    {
        // Printout
        printf("\n\n");
        printf("controlword.switch_on                 = %d\n", controlword.bits.switch_on);
        printf("controlword.enable_voltage            = %d\n", controlword.bits.enable_voltage);
        printf("controlword.quick_stop                = %d\n", controlword.bits.quick_stop);
        printf("controlword.enable_operation          = %d\n", controlword.bits.enable_operation);
        printf("controlword.operation_mode_specific_0 = %d\n", controlword.bits.operation_mode_specific_0);
        printf("controlword.fault_reset               = %d\n", controlword.bits.fault_reset);
        printf("controlword.halt                      = %d\n", controlword.bits.halt);
        printf("controlword.operation_mode_specific_1 = %d\n", controlword.bits.operation_mode_specific_1);
        printf("controlword.manufacturer_specific     = %d\n", controlword.bits.manufacturer_specific);
    }

protected:
    // vector containing all devices
    std::vector<EtherCatDevice*> devices_;

    // map mapping COB id to parse functions
    AddressToFunctionMap addressToFunctionMap_;

    std::vector<std::shared_ptr<Datagram>> datagrams_;



    // EtherCAT Master Data
    /** Main slave data array.
     *  Each slave found on the network gets its own record.
     *  ec_slave[0] is reserved for the master. Structure gets filled
     *  in by the configuration function ec_config().
     */
    ec_slavet        ecat_slave[EC_MAXSLAVE];
    /** number of slaves found on the network */
    int              ecat_slavecount;
    /** slave group structure */
    ec_groupt        ecat_group[EC_MAXGROUP];

    /** cache for EEPROM read functions */
    uint8            ecat_esibuf[EC_MAXEEPBUF];
    /** bitmap for filled cache buffer bytes */
    uint32           ecat_esimap[EC_MAXEEPBITMAP];
    /** current slave for EEPROM cache buffer */
    ec_eringt        ecat_elist;
    ec_idxstackT     ecat_idxstack;

    /** SyncManager Communication Type struct to store data of one slave */
    ec_SMcommtypet   ecat_SMcommtype[EC_MAX_MAPT];
    /** PDO assign struct to store data of one slave */
    ec_PDOassignt    ecat_PDOassign[EC_MAX_MAPT];
    /** PDO description struct to store data of one slave */
    ec_PDOdesct      ecat_PDOdesc[EC_MAX_MAPT];

    /** buffer for EEPROM SM data */
    ec_eepromSMt     ecat_SM;
    /** buffer for EEPROM FMMU data */
    ec_eepromFMMUt   ecat_FMMU;
    /** Global variable TRUE if error available in error stack */
    boolean          ecat_error = FALSE;

    int64            ecat_DCtime;

    ecx_portt        ecat_port;
    ecx_redportt     ecat_redport;


    ecx_contextt ecatContext_;





    ec_ODlistt odinfo_;
    ec_OElistt odentryinfo_;

    const int timeoutmon_ = 500;
    char IOmap_[4096];
    std::atomic<int> wkcExpected_;
    std::atomic<int> wkc_;
    std::atomic<bool> needlf_;
    std::atomic<bool> inOP_;
    std::atomic<int> oloop_;
    std::atomic<int> iloop_;
    uint8 currentgroup_ = 0;

    bool running_ = false;
    elmo_twitter_outdata_t outdata_;
    elmo_twitter_indata_t indata_;
};

} /* namespace tcan */

