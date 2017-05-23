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
#include "tcan_ethercat/EtherCatBusOptions.hpp"
#include "tcan_ethercat/EtherCatSlave.hpp"

namespace tcan_ethercat {


class EtherCatBus : public tcan::Bus<EtherCatDatagrams> {
 public:
    typedef std::function<bool(const EtherCatDatagram&)> TxPdoCallbackPtr;
    typedef std::unordered_map<EtherCatSlave*, TxPdoCallbackPtr> TxPdoCallbackMap;

    EtherCatBus() = delete;
    EtherCatBus(std::unique_ptr<EtherCatBusOptions>&& options)
    : tcan::Bus<EtherCatDatagrams>(std::move(options)),
      wkcExpected_(0),
      wkc_(0) {}

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

    template <class T>
    inline bool addTxPdoCallback(T* device, bool(std::common_type<T>::type::*fp)(const EtherCatDatagram&)) {
        return txPdoCallbackMap_.emplace(device, std::bind(fp, device, std::placeholders::_1)).second;
    }

    bool setupCommunication() {
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

//        if (!allSlavesMatch()) {
//            MELO_ERROR_STREAM("Expected and discovered slaves mismatch.");
//            return false;
//        }
//        MELO_INFO_STREAM("Expected and discovered slaves match.");

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
        ecx_config_map_group(&ecatContext_, &ioMap_, 0);
        printDistributedClockState();
        MELO_INFO_STREAM("Locating and measuring distributed clocks ...");
        if (ecx_configdc(&ecatContext_)) {
            MELO_INFO_STREAM("Distributed clocks located and measured.");
        } else {
            MELO_INFO_STREAM("No distributed clocks located and measured.");
        }
        printDistributedClockState();

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
        strangeFunction();
        checkSlaveErrors();
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

    void setStateInit() { setState(EC_STATE_INIT); }
    void setStatePreOp() { setState(EC_STATE_PRE_OP); }
    void setStateBoot() { setState(EC_STATE_BOOT); }
    void setStateSafeOp() { setState(EC_STATE_SAFE_OP); }
    void setStateOperational() { setState(EC_STATE_OPERATIONAL); }

    void waitForStateInit() { waitForState(EC_STATE_INIT); }
    void waitForStatePreOp() { waitForState(EC_STATE_PRE_OP); }
    void waitForStateBoot() { waitForState(EC_STATE_BOOT); }
    void waitForStateSafeOp() { waitForState(EC_STATE_SAFE_OP); }
    void waitForStateOperational() { waitForState(EC_STATE_OPERATIONAL); }

    void syncDistributedClocks(const uint16_t slave, const bool activate) {
        MELO_INFO_STREAM((activate ? "Activating" : "Deactivating") << " distributed clock synchronization for slave " << slave << " ...")
        const double timeStep = 1e-3;
        ecx_dcsync0(&ecatContext_, slave, static_cast<boolean>(activate), static_cast<uint32>(timeStep*1e9), static_cast<int32>(timeStep*0.5*1e9));
        MELO_INFO_STREAM("Finished " << (activate ? "activating" : "deactivating") << " distributed clock synchronization.")
    }

    void printDistributedClockState() {
        MELO_INFO_STREAM("DC state:");
        MELO_INFO_STREAM("DC time: " << *ecatContext_.DCtime);
        MELO_INFO_STREAM("DC tO: " << ecatContext_.DCtO);
        MELO_INFO_STREAM("DC l: " << ecatContext_.DCl);
        for (int i = 0; i <= *ecatContext_.slavecount; i++) {
            MELO_INFO_STREAM("  Slave " << i << ": " << std::string(ecatContext_.slavelist[i].name));
            MELO_INFO_STREAM("    Has DC: " << static_cast<bool>(ecatContext_.slavelist[i].hasdc));
            MELO_INFO_STREAM("    DC active: " << static_cast<bool>(ecatContext_.slavelist[i].DCactive));
            MELO_INFO_STREAM("    DC cycle: " << ecatContext_.slavelist[i].DCcycle);
            MELO_INFO_STREAM("    DC previous: " << ecatContext_.slavelist[i].DCnext);
            MELO_INFO_STREAM("    DC next: " << ecatContext_.slavelist[i].DCprevious);
            MELO_INFO_STREAM("    DC rt A: " << ecatContext_.slavelist[i].DCrtA);
            MELO_INFO_STREAM("    DC rt B: " << ecatContext_.slavelist[i].DCrtB);
            MELO_INFO_STREAM("    DC rt C: " << ecatContext_.slavelist[i].DCrtC);
            MELO_INFO_STREAM("    DC rt D: " << ecatContext_.slavelist[i].DCrtD);
            MELO_INFO_STREAM("    DC shift: " << ecatContext_.slavelist[i].DCshift);
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

    std::shared_ptr<EtherCatDatagrams> getData() {
        return datagramsSent_;
    }

 protected:/// INTERNAL FUNCTIONS

    /*! Initialize the device driver
     * @return true if successful
     */
    bool initializeInterface() {
        printf("Starting simple test\n");

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

        return true;
    }

    void cleanupInterface() {
//        /* request INIT state for all slaves */
//        printf("\nRequest init state for all slaves\n");
//        ecatContext_.slavelist[0].state = EC_STATE_INIT;
//        ecx_writestate(&ecatContext_, 0);
//
//        /* stop SOEM, close socket */
//        printf("End simple test, close socket\n");
//        if (ecatContext_.port) {
//            ecx_close(&ecatContext_);
//            printf("Closed socket\n");
//        }
//
//        for (auto slave : slaves_) {
//            std::cout << slave << std::endl;
//            delete slave;
//        }
//        printf("Deleted slaves.\n");
    }

    /*! Is called after reception of a message. Routes the message to the callback.
     * @param msg    reference to the ethercat message
     */
    void handleMessage(const EtherCatDatagrams& msg) {

        // TODO:
        // check time the frame took to travel through the physical bus
        // check working counters
        // extract datagrams from ethernet frame and map datagrams to device callback
        for (EtherCatSlave* slave : slaves_) {
            slave->resetDeviceTimeoutCounter();
            auto callback = txPdoCallbackMap_.find(slave);
            if (callback != txPdoCallbackMap_.end()) {
                callback->second(msg.rxAndTxPdoDatagrams_.at(slave->getAddress()).second); //TODO improve access
            }
        }
    }

    void sendProcessData() {
        ecx_send_processdata(&ecatContext_);
    }

    void receiveProcessData() {
        wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
    }

    /*! read CAN message from the device driver
     * @return true if a message was successfully read and parsed
     */
    virtual bool readData() {
        if (!datagramsSent_) {
            MELO_WARN_STREAM("Nothing to read, since no data has been sent yet.");
            return false;
        }

        receiveProcessData();

        if (!workingCounterIsOk()) {
            MELO_WARN_STREAM("Working counter is too low (" << wkc_ << " < " << wkcExpected_ << ").");
            return false;
        }

        for (int i = 1; i <= *ecatContext_.slavecount; i++) {
            memcpy(
                datagramsSent_->rxAndTxPdoDatagrams_[i].second.data_,
                ecatContext_.slavelist[i].inputs,
                datagramsSent_->rxAndTxPdoDatagrams_[i].second.getDataLength());
        }

        handleMessage(*datagramsSent_);

        return false; // TODO true?
    }

    /*! write datagrams to the device driver
     * @return true if the message was successfully written
     */
    virtual bool writeData(std::unique_lock<std::mutex>* lock) {
        EtherCatDatagrams msg = outgoingMsgs_.front();
        if(lock != nullptr) {
            lock->unlock();
        }

        for (const auto& rxAndTxDatagram : msg.rxAndTxPdoDatagrams_) {
            memcpy(
                ecatContext_.slavelist[rxAndTxDatagram.second.first.header_.address_].outputs,
                rxAndTxDatagram.second.first.getData(),
                rxAndTxDatagram.second.first.getDataLength());
        }

        datagramsSent_.reset(new EtherCatDatagrams(msg));

        sendProcessData();

        if(lock != nullptr) {
            lock->lock();
        }
        outgoingMsgs_.pop_front();

        return true;
    }

    /*! Do a sanity check of all devices on this bus.
     */
    void sanityCheck() {
        uint8_t currentgroup = 0;
        if (!workingCounterIsOk() || ecatContext_.grouplist[currentgroup].docheckstate) {
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
                        if (ecx_reconfig_slave(&ecatContext_, slave, EC_TIMEOUTRET)) {
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
                        if (ecx_recover_slave(&ecatContext_, slave, EC_TIMEOUTRET)) {
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

//    inline std::shared_ptr<EtherCatDatagram> addDatagram(EtherCatDatagram&& datagram) {
//        // put the datagram at apropriate position to replicate the physical structure of the bus in the datagram order?
//        auto ptr = std::make_shared<EtherCatDatagram>(std::forward<EtherCatDatagram>(datagram));
//        datagrams_.push_back(ptr);
//        return ptr;
//    }

    bool allSlavesMatch() {
        // Verify the expected number of slaves
        if (*ecatContext_.slavecount < static_cast<int>(slaves_.size())) {
            MELO_ERROR_STREAM("Too few slaves found (" << *ecatContext_.slavecount << " < " << static_cast<int>(slaves_.size()) << ").")
            return false;
        }
        else if (*ecatContext_.slavecount > static_cast<int>(slaves_.size())) {
            MELO_ERROR_STREAM("Too many slaves found (" << *ecatContext_.slavecount << " < " << static_cast<int>(slaves_.size()) << ").")
            return false;
        }

        // Verify slave by slave that the name is correct
//        for (const EtherCatSlave* slave : slaves_) {
//            if (std::string(ecatContext_.slavelist[slave->getAddress()].name) != slave->getName()) {
//                MELO_ERROR_STREAM("Slave with address '" << slave->getAddress() << "' expected name '" << slave->getName() << "' but found '" << std::string(ecatContext_.slavelist[slave->getAddress()].name) << "'.");
//                return false;
//            }
//        }

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
        printf("Slave EtherCAT StateMachine state is 0x%x\n", slavestate);

        int sdodata = 0;
        int sdodatasize = sizeof(int);
        wkc_ = ecx_SDOread(&ecatContext_, 1, 0x6041, 0, 0, &sdodatasize, &sdodata, EC_TIMEOUTRXM);
        printf("Slave DSP402 StateMachine state is 0x%x\n", (uint16)sdodata);
    }

    void checkSlaveErrors() {
        MELO_INFO_STREAM("Error report:");
        if (ecx_iserror(&ecatContext_)) {
            int i = 0;
            do {
                MELO_INFO_STREAM("  Error " << ++i << ": " << ecx_elist2string(&ecatContext_));
            } while(ecx_iserror(&ecatContext_));
        } else {
            MELO_INFO_STREAM("No errors.");
        }
    }

    bool workingCounterIsOk() {
        return wkc_ >= wkcExpected_;
    }

    void strangeFunction() {
        if (ecatContext_.slavelist[0].state == EC_STATE_OPERATIONAL ) {
            printf("Operational state reached for all slaves.\n");
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

    void setState(const uint16_t state) {
        ecatContext_.slavelist[0].state = state;
        ecx_writestate(&ecatContext_, 0);
        MELO_INFO_STREAM("State " << state << " has been set.");
    }

    void waitForState(const uint16_t state) {
        ecx_statecheck(&ecatContext_, 0, state,  EC_TIMEOUTSTATE * 2);
        const unsigned int maxChecks = 40;
        unsigned int check = 0;
        do {
            sendProcessData();
            receiveProcessData();
            ecx_statecheck(&ecatContext_, 0, state,  50000);
            check++;
        } while (check <= maxChecks && (ecatContext_.slavelist[0].state != state));

        if (ecatContext_.slavelist[0].state != state) {
          MELO_WARN_STREAM("State " << state << " has not been reached.");
        } else {
          MELO_INFO_STREAM("State " << state << " has been reached.");
        }
    }

protected:
    // vector containing all slaves
    std::vector<EtherCatSlave*> slaves_;

    // map mapping COB id to parse functions
    TxPdoCallbackMap txPdoCallbackMap_;

    std::shared_ptr<EtherCatDatagrams> datagramsSent_;

    // EtherCAT input/output mapping of the slaves within the datagrams.
    char ioMap_[4096];

    // Working counters.
    std::atomic<int> wkcExpected_;
    std::atomic<int> wkc_;

    // EtherCAT context data elements:

    // Port reference.
    ecx_portt ecatPort_;
    // List of slave data. Index 0 is reserved for the master, higher indices for the slaves.
    ec_slavet ecatSlavelist_[EC_MAXSLAVE];
    // Number of slaves found in the network.
    int ecatSlavecount_ = 0;
    // Slave group structure.
    ec_groupt ecatGrouplist_[EC_MAXGROUP];
    // Internal, reference to eeprom cache buffer.
    uint8 ecatEsiBuf_[EC_MAXEEPBUF];
    // Internal, reference to eeprom cache map.
    uint32 ecatEsiMap_[EC_MAXEEPBITMAP];
    // Internal, reference to error list.
    ec_eringt ecatEList_;
    // Internal, reference to processdata stack buffer info.
    ec_idxstackT ecatIdxStack_;
    // Boolean indicating if an error is available in error stack.
    boolean ecatError_ = FALSE;
    // Reference to last DC time from slaves.
    int64 ecatDcTime_ = 0;
    // Internal, SM buffer.
    ec_SMcommtypet ecatSmCommtype_[EC_MAX_MAPT];
    // Internal, PDO assign list.
    ec_PDOassignt ecatPdoAssign_[EC_MAX_MAPT];
    // Internal, PDO description list.
    ec_PDOdesct ecatPdoDesc_[EC_MAX_MAPT];
    // Internal, SM list from eeprom.
    ec_eepromSMt ecatSm_;
    // Internal, FMMU list from eeprom.
    ec_eepromFMMUt ecatFmmu_;

    // EtherCAT context data.
    ecx_contextt ecatContext_ = {
        &ecatPort_,
        &ecatSlavelist_[0],
        &ecatSlavecount_,
        EC_MAXSLAVE,
        &ecatGrouplist_[0],
        EC_MAXGROUP,
        &ecatEsiBuf_[0],
        &ecatEsiMap_[0],
        0,
        &ecatEList_,
        &ecatIdxStack_,
        &ecatError_,
        0,
        0,
        &ecatDcTime_,
        &ecatSmCommtype_[0],
        &ecatPdoAssign_[0],
        &ecatPdoDesc_[0],
        &ecatSm_,
        &ecatFmmu_,
        nullptr
    };
};

} /* namespace tcan_ethercat */

