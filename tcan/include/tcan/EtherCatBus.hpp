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
#include "tcan/EtherCatSlave.hpp"

namespace tcan {


class EtherCatBus : public Bus<EtherCatDatagrams> {
 public:
    typedef std::function<bool(const EtherCatDatagram&)> CallbackPtr;
    typedef std::unordered_map<uint32_t, std::pair<EtherCatSlave*, CallbackPtr>> AddressToFunctionMap;

    EtherCatBus() = delete;
    EtherCatBus(EtherCatBusOptions* options)
    : Bus<EtherCatDatagrams>(options),
      wkcExpected_(0),
      wkc_(0),
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
        cleanupInterface();
    }

    /*!
     * in-place construction of a new device
     * @param options   pointer to the option class of the device
     * @return true if successful
     */
    template <class C, typename TOptions>
    inline std::pair<C*, bool> addSlave(TOptions* options) {
        C* dev = new C(options);
        bool success = addDevice(dev);
        return std::make_pair(dev, success);
    }

    /*! Adds a slave to the device vector and calls its initDevice function
     * @param device    Pointer to the device
     * @return true if init was successful
     */
    inline bool addSlave(EtherCatSlave* slave) {
        // assign the slave some id to calculate the offset in ethernet frame address
        slaves_.push_back(slave);
        return slave->initDeviceInternal(this);
    }

//    template <class T>
//    inline std::shared_ptr<EtherCatDatagram> addReadDatagram(EtherCatDatagram&& datagram, T* device, bool(std::common_type<T>::type::*fp)()) {
//        // add a datagram linked with a callback to addressToFunctionMap_, use datagram address as key
//        return addDatagram(std::forward<EtherCatDatagram>(datagram));
//    }
//
//    inline std::shared_ptr<EtherCatDatagram> addWriteDatagram(EtherCatDatagram&& datagram) {
//        return addDatagram(std::forward<EtherCatDatagram>(datagram));
//    }

    /*!
     * Should be called from the same thread as write operations on the datagrams, otherwise it is up to the user to ensure thread safety
     * @return  true on success
     */
//    inline void dispatchFrame() {
//        // Compute length of the ethernet frame.
//        uint16_t len = 20; // Ethernet + EtherCat overhead. TODO: derive EthernetFrame from GenericMsg and add length getter
//        for (const auto& datagramPtr : datagrams_) {
//            len += datagramPtr->getTotalLength();
//        }
//
//        // Instantiate data.
//        uint8_t* data = new uint8_t[len];
//        uint16_t pos = 0;
//
//        // Copy ethercat header.
//        data[pos++] = static_cast<uint8_t>((ethercatHeader.data_ >> 0) & 0xFF);
//        data[pos++] = static_cast<uint8_t>((ethercatHeader.data_ >> 8) & 0xFF);
//        // Copy ethercat datagrams.
//        for (const auto& datagramPtr : datagrams_) {
//            // Copy ethercat datagram header.
//            data[pos++] = static_cast<uint8_t>(datagramPtr->header_.cmd_);
//            data[pos++] = static_cast<uint8_t>(datagramPtr->header_.idx_);
//            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.address_ >> 0) & 0xFF);
//            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.address_ >> 8) & 0xFF);
//            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.address_ >> 16) & 0xFF);
//            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.address_ >> 24) & 0xFF);
//            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.lenRCM_.lenRCMAll_ >> 0) & 0xFF);
//            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.lenRCM_.lenRCMAll_ >> 8) & 0xFF);
//            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.irq_ >> 0) & 0xFF);
//            data[pos++] = static_cast<uint8_t>((datagramPtr->header_.irq_ >> 8) & 0xFF);
//            // Copy ethercat datagram data.
//            for (uint16_t i = 0; i < datagramPtr->getDataLength(); i++) {
//                data[pos++] = static_cast<uint8_t>(datagramPtr->data_[i]);
//            }
//            // Copy ethercat datagram working counter.
//            data[pos++] = static_cast<uint8_t>((datagramPtr->workingCounter_ >> 0) & 0xFF);
//            data[pos++] = static_cast<uint8_t>((datagramPtr->workingCounter_ >> 8) & 0xFF);
//        }
//
//        // compute expected working counter
//
//        // other option:
//        EthernetFrame msg;
//        msg.emplaceData(len, data);
//        sendMessage(msg);
//    }

    void discoverDevices() {
        // do not add the devices by hand but get the topology and memory from the connected devices?
    }

 public:/// INTERNAL FUNCTIONS
    /*! Is called after reception of a message. Routes the message to the callback.
     * @param cmsg    reference to the can message
     */
    void handleMessage(const EtherCatDatagrams& msg) {

        // TODO:
        // check time the frame took to travel through the physical bus
        // check working counters
        // extract datagrams from ethernet frame and map datagrams to device callback
    }

    /*! Initialize the device driver
     * @return true if successful
     */
    virtual bool initializeInterface() {
        printf("Starting simple test\n");
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

        if (!allSlavesMatch()) {
            MELO_ERROR_STREAM("Expected and discovered slaves mismatch.");
            return false;
        }
        MELO_INFO_STREAM("Expected and discovered slaves match.");

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
        ecx_configdc(&ecatContext_);

        printf("Slaves mapped, state to SAFE_OP.\n");
        /* wait for all slaves to reach SAFE_OP state */
        if (ecx_statecheck(&ecatContext_, 0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4) != EC_STATE_SAFE_OP) {
            printf("State EC_STATE_SAFE_OP has not been reached.\n");
        }

//        int oloop_ = ecatContext_.slavelist[0].Obytes;
//        if ((oloop_ == 0) && (ecatContext_.slavelist[0].Obits > 0)) oloop_ = 1;
////        if (oloop_ > 8) oloop_ = 8;
//        int iloop_ = ecatContext_.slavelist[0].Ibytes;
//        if ((iloop_ == 0) && (ecatContext_.slavelist[0].Ibits > 0)) iloop_ = 1;
////        if (iloop_ > 8) iloop_ = 8;

        printf("segments : %d : %d %d %d %d\n",
            ecatContext_.grouplist[0].nsegments,
            ecatContext_.grouplist[0].IOsegment[0],
            ecatContext_.grouplist[0].IOsegment[1],
            ecatContext_.grouplist[0].IOsegment[2],
            ecatContext_.grouplist[0].IOsegment[3]);

        // Disable symmetrical transfers
        ecatContext_.grouplist[0].blockLRW = 1;

        wkcExpected_ = (ecatContext_.grouplist[0].outputsWKC * 2) + ecatContext_.grouplist[0].inputsWKC;
        printf("Calculated workcounter %d\n", wkcExpected_.load());

        // Initialize the communication interfaces of all slaves.
        checkSlaveStates();
        for (EtherCatSlave* slave : slaves_) {
            if (!slave->initializeInterface()) {
                MELO_ERROR_STREAM("Slave '" << slave->getName() << "' was not initialized successfully.");
                return false;
            }
        }
        checkSlaveStates();
        checkSlaveErrors();


        strangeFunction();

        return true;
    }

    void cleanupInterface() {
        /* request INIT state for all slaves */
        printf("\nRequest init state for all slaves\n");
        ecatContext_.slavelist[0].state = EC_STATE_INIT;
        ecx_writestate(&ecatContext_, 0);

        /* stop SOEM, close socket */
        printf("End simple test, close socket\n");
        ecx_close(&ecatContext_);
        printf("Closed socket\n");
    }

    /*! read CAN message from the device driver
     * @return true if a message was successfully read and parsed
     */
    virtual bool readData() {
        if (!datagramsSent_) {
            MELO_WARN_STREAM("No data has been sent yet.");
            return false;
        }

        receiveProcessData();

        if (!checkWorkingCounter()) {
            MELO_WARN_STREAM("Working counter is too low (" << wkc_ << " < " << wkcExpected_ << ").");
            return false;
        }

        for (int i = 1; i <= *ecatContext_.slavecount; i++) {
            memcpy(
                datagramsSent_->rxAndTxPdoDatagrams_[i].second.data_,
                ecatContext_.slavelist[i].inputs,
                datagramsSent_->rxAndTxPdoDatagrams_[i].second.getDataLength());
        }

        // TODO fill this.
//        EtherCatDatagrams etherCatDatagrams;
//        handleMessage(etherCatDatagrams);

        return false; // TODO true?
    }

    /*! write datagrams to the device driver
     * @return true if the message was successfully written
     */
    virtual bool writeData(const EtherCatDatagrams& msg) {

        for (const auto& rxAndTxDatagram : msg.rxAndTxPdoDatagrams_) {
            memcpy(
                ecatContext_.slavelist[rxAndTxDatagram.second.first.header_.address_].outputs,
                rxAndTxDatagram.second.first.getData(),
                rxAndTxDatagram.second.first.getDataLength());
        }

        datagramsSent_.reset(new EtherCatDatagrams(msg));

        sendProcessData();

        return true;
    }

    /*! Do a sanity check of all devices on this bus.
     */
    void sanityCheck() {
        uint8_t currentgroup = 0;
        if (inOP_ && (checkWorkingCounter() || ecatContext_.grouplist[currentgroup].docheckstate)) {
            /* one ore more slaves are not responding */
            ecatContext_.grouplist[currentgroup].docheckstate = FALSE;
            ecx_readstate(&ecatContext_);
            for (int slave = 1; slave <= *ecatContext_.slavecount; slave++) {
                if ((ecatContext_.slavelist[slave].group == currentgroup) && (ecatContext_.slavelist[slave].state != EC_STATE_OPERATIONAL)) {
                    ecatContext_.grouplist[currentgroup].docheckstate = TRUE;
                    if (ecatContext_.slavelist[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ecatContext_.slavelist[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ecx_writestate(&ecatContext_, slave);
                    }
                    else if (ecatContext_.slavelist[slave].state == EC_STATE_SAFE_OP) {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ecatContext_.slavelist[slave].state = EC_STATE_OPERATIONAL;
                        ecx_writestate(&ecatContext_, slave);
                    }
                    else if (ecatContext_.slavelist[slave].state > 0) {
                        if (ecx_reconfig_slave(&ecatContext_, slave, timeoutmon_)) {
                            ecatContext_.slavelist[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n",slave);
                        }
                    }
                    else if (!ecatContext_.slavelist[slave].islost) {
                        /* re-check state */
                        ecx_statecheck(&ecatContext_, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ecatContext_.slavelist[slave].state) {
                            ecatContext_.slavelist[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n",slave);
                        }
                    }
                }
                if (ecatContext_.slavelist[slave].islost) {
                    if (!ecatContext_.slavelist[slave].state) {
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
            if (!ecatContext_.grouplist[currentgroup].docheckstate) {
                printf("OK : all slaves resumed OPERATIONAL.\n");
            }
        }






        bool isMissingOrError = false;
        bool allActive = true;
        for (auto slave : slaves_) {
            slave->sanityCheck();
            isMissingOrError |= slave->isMissing() | slave->hasError();
            allActive &= slave->isActive();
        }

        isMissingDeviceOrHasError_ = isMissingOrError;
        allDevicesActive_ = allActive;
    }

    std::shared_ptr<EtherCatDatagrams> getData() {
        return datagramsSent_;
    }

//    inline std::shared_ptr<EtherCatDatagram> addDatagram(EtherCatDatagram&& datagram) {
//        // put the datagram at apropriate position to replicate the physical structure of the bus in the datagram order?
//        auto ptr = std::make_shared<EtherCatDatagram>(std::forward<EtherCatDatagram>(datagram));
//        datagrams_.push_back(ptr);
//        return ptr;
//    }

    bool allSlavesMatch() {
        // Verify the expected number of slaves
        if (*ecatContext_.slavecount < static_cast<int>(slaves_.size())) {
            MELO_ERROR_STREAM("Found too few slaves (" << *ecatContext_.slavecount << " < " << static_cast<int>(slaves_.size()) << ").")
            return false;
        }
        else if (*ecatContext_.slavecount > static_cast<int>(slaves_.size())) {
          MELO_ERROR_STREAM("Found too many slaves (" << *ecatContext_.slavecount << " < " << static_cast<int>(slaves_.size()) << ").")
            return false;
        }

        // Verify slave by slave that the name is correct
        for (const EtherCatSlave* slave : slaves_) {
            if (std::string(ecatContext_.slavelist[slave->getAddress()].name) != slave->getName()) {
                MELO_ERROR_STREAM("Slave with address '" << slave->getAddress() << "' expected name '" << slave->getName() << "' but found '" << std::string(ecatContext_.slavelist[slave->getAddress()].name) << "'.");
                return false;
            }
        }

        return true;
    }

    void getSlaveInfo() {
        int ret = 0;
        int Osize = 0, Isize = 0;

        #define Nsdo  16
        int sdodata[Nsdo], sdodatasize;
        char *databuf;
        databuf = (char*)&sdodata[0];

        // Check PDO mapping size
        ret = ecx_readPDOmap(&ecatContext_, 1, &Osize, &Isize);
        printf("\n\nec_readPDOmap returned %d\n", ret);
        printf("Osize in bits = %d\n", Osize);
        printf("Isize in bits = %d\n", Isize);

        // Get complete OD dump
        ec_ODlistt odinfo;
        ec_OElistt odentryinfo;
        ret = ecx_readODlist(&ecatContext_, 1, &odinfo);
        printf("\nc_readODlist returned %d\n", ret);
        printf("Slave = %d, Entries = %d\n", odinfo.Slave, odinfo.Entries);
        for (int k = 0; k < odinfo.Entries; k++) {
            ecx_readODdescription(&ecatContext_, k, &odinfo);
            ecx_readOE(&ecatContext_, k, &odinfo, &odentryinfo);
            printf("\nIndex = 0x%x\n", odinfo.Index[k]);
            printf("    MaxSub     = %d\n", odinfo.MaxSub[k]+1);
            printf("    ObjectCode = %d\n", odinfo.ObjectCode[k]);
            printf("    DataType   = %d\n", odinfo.DataType[k]);
            printf("    Description: %s\n", &odinfo.Name[k][0]);
            printf("    OE Entries = %d\n", odentryinfo.Entries);
            for (int j = 0; j < odentryinfo.Entries; j++) {
                for (int n = 0; n < Nsdo; n++) {
                    sdodata[n] = 0;
                }
                sdodatasize = Nsdo*sizeof(int);
                ecx_SDOread(&ecatContext_, odinfo.Slave, odinfo.Index[k], j, 0, &sdodatasize, &sdodata, EC_TIMEOUTRXM);
                printf("    OE = %d\n", j);
                printf("        ValueInfo  = %d\n", odentryinfo.ValueInfo[j]);
                printf("        DataType   = %d\n", odentryinfo.DataType[j]);
                printf("        BitLength  = %d\n", odentryinfo.BitLength[j]);
                printf("        ObjAccess  = %d\n", odentryinfo.ObjAccess[j]);
                printf("        Name       = %s\n", &odentryinfo.Name[j][0]);
                printf("        Value      =");
                for (int n=0; n<sdodatasize; n++) {
                    printf(" 0x%x", (0xFF & databuf[n]));
                }
                printf("\n");
            }
        }
    }

    void checkSlaveStates() {
        int slavestate = ecx_readstate(&ecatContext_);
        printf("\nSlave EtherCAT StateMachine state is 0x%x\n", slavestate);

        int sdodata = 0;
        int sdodatasize = sizeof(int);
        wkc_ = ecx_SDOread(&ecatContext_, 1, 0x6041, 0, 0, &sdodatasize, &sdodata, EC_TIMEOUTRXM);
        printf("Slave DSP402 StateMachine state is 0x%x\n", (uint16)sdodata);
    }

    void checkSlaveErrors() {
        printf("\nERROR REPORT:\n");
        if (ecx_iserror(&ecatContext_)) {
            int i = 0;
            do {
                char* errstr = ecx_elist2string(&ecatContext_);
                printf("    Error %d: %s", ++i, errstr);
            } while(ecx_iserror(&ecatContext_));
        }
    }

    template <typename Value>
    void sendSdoWrite(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value) {
        const int size = sizeof(Value);
        Value valueCopy = value; // copy value to make it modifiable
        wkc_ = ecx_SDOwrite(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), size, &valueCopy, EC_TIMEOUTRXM);
    }

    template <typename Value>
    void sendSdoRead(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value) {
        int size = sizeof(Value);
        wkc_ = ecx_SDOread(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), &size, &value, EC_TIMEOUTRXM);
        if (size != sizeof(Value)) {
            MELO_WARN_STREAM("Expected (" << sizeof(Value) << ") and read (" << size << ") SDO value size mismatch.");
        }
    }

    void sendSdoReadAndPrint(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess) {
        int sdodata[4]={0,0,0,0};
        sendSdoRead(slave, index, subindex, completeAccess, sdodata);
        printf(" OD entry {Index 0x%x, Subindex 0x%x} is  0x%x 0x%x 0x%x 0x%x\n", index, subindex, sdodata[3], sdodata[2], sdodata[1], sdodata[0]);
    }

    void syncDistributedClocks(const uint16_t slave, const bool activate) {
        MELO_INFO_STREAM("Starting synchronizing clocks ...")
        const double timeStep = 1e-3;
        ecx_dcsync0(&ecatContext_, slave, static_cast<boolean>(activate), (int64)(timeStep*1e9), (int64)(timeStep*0.5*1e9));
        MELO_INFO_STREAM("Finished synchronizing clocks.")
    }

    void sendProcessData() {
        ecx_send_processdata(&ecatContext_);
    }

    void receiveProcessData() {
        wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
    }

    bool checkWorkingCounter() {
        return wkc_ >= wkcExpected_;
    }

    void setStatePreOp() { setState(EC_STATE_PRE_OP); }
    void setStateSafeOp() { setState(EC_STATE_SAFE_OP); }
    void setStateOperational() { setState(EC_STATE_OPERATIONAL); }

    void waitForStateOperational() { waitForState(EC_STATE_OPERATIONAL); }

    void strangeFunction() {
        if (ecatContext_.slavelist[0].state == EC_STATE_OPERATIONAL ) {
            printf("Operational state reached for all slaves.\n");
            inOP_ = true;

      //            // Handle PDO streams
      //            outdata_.controlword.all = 0;
      //            ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
      //            ecatcomm_slave_set_rxpdo(&outdata_, SHUTDOWN, 0.0);
      //            ecx_send_processdata(&ecatContext_);
      //            wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
      //            ecatcomm_slave_get_txpdo(&indata_);
      //            osal_usleep(1000);
        }
        else {
            printf("Not all slaves reached operational state.\n");
            ecx_readstate(&ecatContext_);
            for(int i = 1; i <= *ecatContext_.slavecount; i++) {
                if (ecatContext_.slavelist[i].state != EC_STATE_OPERATIONAL) {
                    printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                    i, ecatContext_.slavelist[i].state, ecatContext_.slavelist[i].ALstatuscode, ec_ALstatuscode2string(ecatContext_.slavelist[i].ALstatuscode));
                }
            }
        }
    }

 protected:
    void setState(const uint16_t state) {
        ecatContext_.slavelist[0].state = state;
        ecx_writestate(&ecatContext_, 0);
        ecx_statecheck(&ecatContext_, 0, state,  EC_TIMEOUTSTATE * 4);
    }

    void waitForState(const uint16_t state) {
        int chk = 40;
        do {
            sendProcessData();
            receiveProcessData();
            ecx_statecheck(&ecatContext_, 0, state,  50000);
        } while (chk-- && (ecatContext_.slavelist[0].state != state));
    }

protected:
    // vector containing all slaves
    std::vector<EtherCatSlave*> slaves_;

    // map mapping COB id to parse functions
    AddressToFunctionMap addressToFunctionMap_;

    std::shared_ptr<EtherCatDatagrams> datagramsSent_;

    const int timeoutmon_ = 500;
    char IOmap_[4096];
    std::atomic<int> wkcExpected_;
    std::atomic<int> wkc_;
    std::atomic<bool> inOP_;

    // EtherCAT Master Data TODO: make ecx_contextt contain its own data
    /** Main slave data array.
     *  Each slave found on the network gets its own record.
     *  ec_slave[0] is reserved for the master. Structure gets filled
     *  in by the configuration function ec_config().
     */
    ecx_portt        ecat_port;
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
    /** Global variable TRUE if error available in error stack */
    boolean          ecat_error = FALSE;
    int64            ecat_DCtime;

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

//    ecx_redportt     ecat_redport;

    ecx_contextt ecatContext_;
};

} /* namespace tcan */

