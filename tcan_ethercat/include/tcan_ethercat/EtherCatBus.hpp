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
#include <iomanip>

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

    EtherCatBus(std::unique_ptr<EtherCatBusOptions>&& options)
    : tcan::Bus<EtherCatDatagrams>(std::move(options)),
      wkcExpected_(0),
      wkc_(0) {
      // Initialize all SOEM context data pointers that are not used with null.
      ecatContext_.port->stack.sock = nullptr;
      ecatContext_.port->stack.txbuf = nullptr;
      ecatContext_.port->stack.txbuflength = nullptr;
      ecatContext_.port->stack.tempbuf = nullptr;
      ecatContext_.port->stack.rxbuf = nullptr;
      ecatContext_.port->stack.rxbufstat = nullptr;
      ecatContext_.port->stack.rxsa = nullptr;
      ecatContext_.port->redport = nullptr;
//      ecatContext_.idxstack->data = nullptr; // This does not compile since they use a fixed size array of void pointers ...
      ecatContext_.FOEhook = nullptr;
    }

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

    template <class T>
    inline bool addTxPdoCallback(T* device, bool(std::common_type<T>::type::*fp)(const EtherCatDatagram&)) {
        return txPdoCallbackMap_.emplace(device, std::bind(fp, device, std::placeholders::_1)).second;
    }

    bool setupCommunication() {
        // Initialize SOEM.
        if (ecx_config_init(&ecatContext_, FALSE) == 0) {
            MELO_ERROR_STREAM("No slaves have been found.");
            return false;
        }

        // Print the slaves which have been detected.
        MELO_INFO_STREAM("Bus '" << options_->name_ << "': The following " << *ecatContext_.slavecount << " slaves have been found and configured:");
        for (int i = 1; i <= *ecatContext_.slavecount; i++) {
            MELO_INFO_STREAM(i << ": " << std::string(ecatContext_.slavelist[i].name));
        }

        // In order to set up the mapping correctly, only continue if all slaves have been found.
        if (!allSlavesMatch()) {
            MELO_ERROR_STREAM("Bus '" << options_->name_ << "': Expected and discovered slaves mismatch.");
            return false;
        }
        MELO_INFO_STREAM("Bus '" << options_->name_ << "': Expected and discovered slaves match.");

        // Disable symmetrical transfers.
        ecatContext_.grouplist[0].blockLRW = static_cast<const EtherCatBusOptions*>(getOptions())->blockLrw_ ? 1 : 0;

        // Initialize the communication interfaces of all slaves.
        for (EtherCatSlave* slave : slaves_) {
            if (!slave->initializeInterface()) {
                MELO_ERROR_STREAM("Bus '" << options_->name_ << "': Slave '" << slave->getName() << "' was not initialized successfully.");
                return false;
            }
        }

        // Set up the communication IO mapping.
        ecx_config_map_group(&ecatContext_, &ioMap_, 0);

        // Configure distributed clocks.
        MELO_INFO_STREAM("Bus '" << options_->name_ << "': Locating and measuring distributed clocks ...");
        if (ecx_configdc(&ecatContext_)) {
            MELO_INFO_STREAM("Bus '" << options_->name_ << "': Distributed clocks located and measured.");
        } else {
            MELO_INFO_STREAM("Bus '" << options_->name_ << "': No distributed clocks located and measured.");
        }

        // Calculate the expected working counter.
        wkcExpected_ = (ecatContext_.grouplist[0].outputsWKC * 2) + ecatContext_.grouplist[0].inputsWKC;
        MELO_INFO_STREAM("Calculated expected working counter: " << wkcExpected_.load());

        // Go to state Safe Op.
        for (EtherCatSlave* slave : slaves_) {
            setStateSafeOp(slave->getAddress());
            if (!waitForStateSafeOp(slave->getAddress())) {
                MELO_ERROR_STREAM("Bus '" << options_->name_ << "': Slave " << slave->getAddress() << " did not reach the safe-op state.");
                return false;
            }
        }

        // Go to state Operational.
        for (EtherCatSlave* slave : slaves_) {
            setStateOperational(slave->getAddress());
            if (!waitForStateOperational(slave->getAddress())) {
                MELO_ERROR_STREAM("Bus '" << options_->name_ << "': Slave " << slave->getAddress() << " did not reach the operational state.");
                return false;
            }
        }

        // Print slave states and errors.
        printSlaveStates();
        printSlaveErrors();

        return true;
    }

    void setStateInit(const uint16_t slave = 0) { setState(EC_STATE_INIT, slave); }
    void setStatePreOp(const uint16_t slave = 0) { setState(EC_STATE_PRE_OP, slave); }
    void setStateBoot(const uint16_t slave = 0) { setState(EC_STATE_BOOT, slave); }
    void setStateSafeOp(const uint16_t slave = 0) { setState(EC_STATE_SAFE_OP, slave); }
    void setStateOperational(const uint16_t slave = 0) { setState(EC_STATE_OPERATIONAL, slave); }

    bool waitForStateInit(const uint16_t slave = 0) { return waitForState(EC_STATE_INIT, slave); }
    bool waitForStatePreOp(const uint16_t slave = 0) { return waitForState(EC_STATE_PRE_OP, slave); }
    bool waitForStateBoot(const uint16_t slave = 0) { return waitForState(EC_STATE_BOOT, slave); }
    bool waitForStateSafeOp(const uint16_t slave = 0) { return waitForState(EC_STATE_SAFE_OP, slave); }
    bool waitForStateOperational(const uint16_t slave = 0) { return waitForState(EC_STATE_OPERATIONAL, slave); }

    void syncDistributedClocks(const uint16_t slave, const bool activate) {
        MELO_INFO_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": " << (activate ? "Activating" : "Deactivating") << " distributed clock synchronization ...")
        const double timeStep = 1e-3;
        ecx_dcsync0(&ecatContext_, slave, static_cast<boolean>(activate), static_cast<uint32>(timeStep*1e9), static_cast<int32>(timeStep*0.5*1e9));
        MELO_INFO_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": Finished " << (activate ? "activating" : "deactivating") << " distributed clock synchronization.")
    }

    void printDistributedClockState() {
        MELO_INFO_STREAM("Bus '" << options_->name_ << "': DC state:");
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
            MELO_WARN_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": Expected (" << sizeof(Value) << ") and read (" << size << ") SDO value size mismatch.");
        }
    }

    void sendMessage(const uint16_t slave, const std::pair<EtherCatDatagram, EtherCatDatagram>& rxAndTxPdoDatagram) {
        // Create a new staged datagrams object if not existing yet.
        if (!stagedDatagrams_) {
            stagedDatagrams_.reset(new tcan_ethercat::EtherCatDatagrams());
        }

        // Stage the Rx and Tx datagram for one slave.
        stagedDatagrams_->rxAndTxPdoDatagrams_.insert({slave, rxAndTxPdoDatagram});

        // Only send the message once the Rx and Tx datagrams of all slaves have been staged.
        if (stagedDatagrams_->rxAndTxPdoDatagrams_.size() < ecatSlavecount_) {
            return;
        }

        // Send all datagrams at once and clear the staged datagrams.
        tcan::Bus<EtherCatDatagrams>::sendMessage(*stagedDatagrams_); // TODO: Use return bool.
        stagedDatagrams_.reset();
    }

    // TODO: Implement emplace method.
//    void emplaceMessage(const uint16_t slave, const std::pair<EtherCatDatagram, EtherCatDatagram>& rxAndTxPdoDatagram) {}

    std::shared_ptr<EtherCatDatagrams> getData() {
        return sentDatagrams_;
    }

 protected:/// INTERNAL FUNCTIONS

    /*! Initialize the device driver
     *  @return true if successful
     */
    bool initializeInterface() {
        /*
         * Followed by start of the application we need to set up the NIC to be used as
         * EtherCAT Ethernet interface. In a simple setup we call ec_init(ifname) and if
         * SOEM comes with support for cable redundancy we call ec_init_redundant that
         * will open a second port as backup. You can send NULL as ifname if you have a
         * dedicated NIC selected in the nicdrv.c. It returns >0 if succeeded.
         */
        const char* ifname = options_->name_.c_str();
        if (ecx_init(&ecatContext_, ifname) <= 0) {
            MELO_ERROR_STREAM("Bus '" << options_->name_ << "': No socket connection. Execute as root.");
            return false;
        }

        MELO_INFO_STREAM("Bus '" << options_->name_ << "': EtherCAT initialization on succeeded.");

        return true;
    }

    /*! Cleanup the interface.
     */
    void cleanupInterface() {
        MELO_INFO_STREAM("Bus '" << options_->name_ << "': Closing socket ...");
        if (ecatContext_.port) {
            ecx_close(&ecatContext_);
        }

        MELO_INFO_STREAM("Bus '" << options_->name_ << "': Deleting slaves ...");
        for (auto slave : slaves_) {
            delete slave;
        }
    }

    /*! Is called after reception of a message. Routes the message to the callback.
     *  @param msg    reference to the ethercat datagram
     */
    void handleMessage(const EtherCatDatagrams& msg) {
        // TODO:
        // check time the frame took to travel through the physical bus
        for (EtherCatSlave* slave : slaves_) {
            slave->resetDeviceTimeoutCounter();
            auto callback = txPdoCallbackMap_.find(slave);
            if (callback != txPdoCallbackMap_.end()) {
                callback->second(msg.rxAndTxPdoDatagrams_.at(slave->getAddress()).second); //TODO improve access
            }
        }
    }

    /*! Send EtherCAT process data.
     */
    void sendProcessData() {
        ecx_send_processdata(&ecatContext_);
    }

    /*! Receive EtherCAT process data and update the working counter.
     */
    void receiveProcessData() {
        wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
    }

    /*! Read datagrams from the device driver.
     *  @return true if a message was successfully read and parsed.
     */
    virtual bool readData() {
        if (!sentDatagrams_) {
            MELO_WARN_STREAM("Bus '" << options_->name_ << "': Nothing to read, since no data has been sent yet.");
            return false;
        }

        receiveProcessData();

        if (!workingCounterIsOk()) {
            MELO_WARN_STREAM("Bus '" << options_->name_ << "': Working counter is too low (" << wkc_ << " < " << wkcExpected_ << ").");
            return false;
        }

        for (int i = 1; i <= *ecatContext_.slavecount; i++) {
            memcpy(
                sentDatagrams_->rxAndTxPdoDatagrams_[i].second.data_,
                ecatContext_.slavelist[i].inputs,
                sentDatagrams_->rxAndTxPdoDatagrams_[i].second.getDataLength());
        }

        handleMessage(*sentDatagrams_);

        return false; // TODO true?
    }

    /*! Write datagrams to the device driver.
     *  @return true if the message was successfully written.
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

        sentDatagrams_.reset(new EtherCatDatagrams(msg));

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
        // TODO: Restructure and use data from SOEM.
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

    bool allSlavesMatch() {
        // Verify the expected number of slaves
        if (*ecatContext_.slavecount < static_cast<int>(slaves_.size())) {
            MELO_ERROR_STREAM("Bus '" << options_->name_ << "': Too few slaves found (" << *ecatContext_.slavecount << " < " << static_cast<int>(slaves_.size()) << ").")
            return false;
        }
        else if (*ecatContext_.slavecount > static_cast<int>(slaves_.size())) {
            MELO_ERROR_STREAM("Bus '" << options_->name_ << "': Too many slaves found (" << *ecatContext_.slavecount << " > " << static_cast<int>(slaves_.size()) << ").")
            return false;
        }

        /*
        // Verify slave by slave that the name is correct
        for (const EtherCatSlave* slave : slaves_) {
            if (std::string(ecatContext_.slavelist[slave->getAddress()].name) != slave->getName()) {
                MELO_ERROR_STREAM("Slave with address '" << slave->getAddress() << "' expected name '" << slave->getName() << "' but found '" << std::string(ecatContext_.slavelist[slave->getAddress()].name) << "'.");
                return false;
            }
        }
        */

        return true;
    }

    void printSlaveInfo() {
        int ret = 0;
        int Osize = 0, Isize = 0;

        #define Nsdo  16
        int sdodata[Nsdo], sdodatasize;
        char *databuf;
        databuf = (char*)&sdodata[0];

        // Check PDO mapping size
        ret = ecx_readPDOmap(&ecatContext_, 1, &Osize, &Isize);
        MELO_INFO_STREAM("ec_readPDOmap returned " << ret);
        MELO_INFO_STREAM("Osize in bits = " << Osize);
        MELO_INFO_STREAM("Isize in bits = " << Isize);

        // Get complete OD dump
        ec_ODlistt odinfo;
        ec_OElistt odentryinfo;
        ret = ecx_readODlist(&ecatContext_, 1, &odinfo);
        MELO_INFO_STREAM("c_readODlist returned " << ret);
        MELO_INFO_STREAM("Slave = " << odinfo.Slave << ", Entries = " << odinfo.Entries);
        for (int k = 0; k < odinfo.Entries; k++) {
            ecx_readODdescription(&ecatContext_, k, &odinfo);
            ecx_readOE(&ecatContext_, k, &odinfo, &odentryinfo);
            MELO_INFO_STREAM("Index = 0x" << std::hex << odinfo.Index[k]);
            MELO_INFO_STREAM("    MaxSub     = " << odinfo.MaxSub[k]+1);
            MELO_INFO_STREAM("    ObjectCode = " << odinfo.ObjectCode[k]);
            MELO_INFO_STREAM("    DataType   = " << odinfo.DataType[k]);
            MELO_INFO_STREAM("    Description: " << &odinfo.Name[k][0]);
            MELO_INFO_STREAM("    OE Entries = " << odentryinfo.Entries);
            for (int j = 0; j < odentryinfo.Entries; j++) {
                for (int n = 0; n < Nsdo; n++) {
                    sdodata[n] = 0;
                }
                sdodatasize = Nsdo*sizeof(int);
                ecx_SDOread(&ecatContext_, odinfo.Slave, odinfo.Index[k], j, 0, &sdodatasize, &sdodata, EC_TIMEOUTRXM);
                MELO_INFO_STREAM("    OE = " << j);
                MELO_INFO_STREAM("        ValueInfo  = " << odentryinfo.ValueInfo[j]);
                MELO_INFO_STREAM("        DataType   = " << odentryinfo.DataType[j]);
                MELO_INFO_STREAM("        BitLength  = " << odentryinfo.BitLength[j]);
                MELO_INFO_STREAM("        ObjAccess  = " << odentryinfo.ObjAccess[j]);
                MELO_INFO_STREAM("        Name       = " << &odentryinfo.Name[j][0]);
                std::stringstream stream;
                stream << "        Value      =" << std::hex;
                for (int n=0; n<sdodatasize; n++) {
                    stream << " 0x" << (0xFF & databuf[n]);
                }
                MELO_INFO_STREAM(stream.str());
            }
        }
    }

    void printSlaveStates() {
        for (uint16_t i = 0; i <= *ecatContext_.slavecount; i++) {
            uint32_t state = 0;
            sendSdoRead(i, 0x6041, 0, false, state);
            MELO_INFO_STREAM("Bus '" << options_->name_ << "': Slave " << i << " state is 0x" << std::hex << state);
        }
    }

    void printSlaveErrors() {
        MELO_INFO_STREAM("Bus '" << options_->name_ << "': Error report:");
        if (ecx_iserror(&ecatContext_)) {
            int i = 0;
            do {
                MELO_INFO_STREAM("  Error " << ++i << ": " << ecx_elist2string(&ecatContext_));
            } while(ecx_iserror(&ecatContext_));
        } else {
            MELO_INFO_STREAM("No errors.");
        }
    }

    void printIoSegments() {
        MELO_INFO_STREAM("IO Segments: " <<
                ecatContext_.grouplist[0].nsegments << ": " <<
                ecatContext_.grouplist[0].IOsegment[0] << ", " <<
                ecatContext_.grouplist[0].IOsegment[1] << ", " <<
                ecatContext_.grouplist[0].IOsegment[2] << ", " <<
                ecatContext_.grouplist[0].IOsegment[3]);
    }

    /*! Check if the working counter is big enough.
     *  @return    true if working counter is equal or higher than expected.
     */
    bool workingCounterIsOk() {
        return wkc_ >= wkcExpected_;
    }

    /*! Set the desired EtherCAT state machine state.
     *  @param state    desired state.
     *  @param slave    slave id, 0 means all slaves.
     */
    void setState(const uint16_t state, const uint16_t slave = 0) {
        ecatContext_.slavelist[slave].state = state;
        ecx_writestate(&ecatContext_, slave);
        MELO_INFO_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": State " << state << " has been set.");
    }

    /*! Wait for an EtherCAT state machine state to be reached.
     *  @param state    desired state.
     *  @param slave    slave id, 0 means all slaves.
     *  @return         true if it the state has been reached within the timeout.
     */
    bool waitForState(const uint16_t state, const uint16_t slave = 0) {
        ecx_statecheck(&ecatContext_, slave, state,  EC_TIMEOUTSTATE * 2);
        const unsigned int maxChecks = 40;
        unsigned int check = 0;
        do {
            sendProcessData();
            receiveProcessData();
            ecx_statecheck(&ecatContext_, slave, state,  50000);
            check++;
        } while (check <= maxChecks && (ecatContext_.slavelist[slave].state != state));

        if (ecatContext_.slavelist[slave].state == state) {
            MELO_INFO_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": State " << state << " has been reached.");
            return true;
        } else {
            MELO_WARN_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": State " << state << " has not been reached.");
            return false;
        }
    }

protected:
    // vector containing all slaves
    std::vector<EtherCatSlave*> slaves_;

    // map mapping COB id to parse functions
    TxPdoCallbackMap txPdoCallbackMap_;

    std::shared_ptr<EtherCatDatagrams> stagedDatagrams_;
    std::shared_ptr<EtherCatDatagrams> sentDatagrams_;

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
    // Internal, reference to EEPROM cache buffer.
    uint8 ecatEsiBuf_[EC_MAXEEPBUF];
    // Internal, reference to EEPROM cache map.
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
    // Internal, SM list from EEPROM.
    ec_eepromSMt ecatSm_;
    // Internal, FMMU list from EEPROM.
    ec_eepromFMMUt ecatFmmu_;

    // EtherCAT context data.
    // Note: SOEM does not use dynamic memory (new/delete). Therefore
    // all context pointers must be null or point to an existing member.
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

