/*
 * EtherCatBus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Remo Diethelm, Philipp Leemann
 *
 * THIS IMPLEMENTATION IS INCOMPLETE! It does not support the full sanityCheck() capabilities.
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

#include <soem/soem/ethercat.h>

#include "tcan/Bus.hpp"
#include "tcan_ethercat/EtherCatBusOptions.hpp"
#include "tcan_ethercat/EtherCatSlave.hpp"

namespace tcan_ethercat {

class EtherCatBus : public tcan::Bus<EtherCatDatagrams> {
 public:
    typedef std::function<bool(const EtherCatDatagram&)> TxPdoCallbackPtr;
    typedef std::unordered_map<EtherCatSlave*, TxPdoCallbackPtr> TxPdoCallbackMap;

    /*!
     * Constructor.
     * @param options Pointer to the EtherCAT bus options.
     */
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
//      ecatContext_.idxstack->data = nullptr; // This does not compile since SOEM uses a fixed size array of void pointers.
      ecatContext_.FOEhook = nullptr;
    }

    /*!
     * Destructor.
     */
    virtual ~EtherCatBus() {
        cleanupInterface();
    }

    /*!
     * Check if a bus is available.
     * @param name Name of the bus.
     * @return True if available.
     */
    static bool BusIsAvailable(const std::string& name) {
        ec_adaptert* adapter = ec_find_adapters();
        while (adapter != nullptr) {
            if (name == std::string(adapter->name)) {
                return true;
            }
            adapter = adapter->next;
        }
        return false;
    }

    /*!
     * Check if this bus is available.
     * @return True if available.
     */
    bool busIsAvailable() {
        return BusIsAvailable(options_->name_);
    }

    /*!
     * Print all available busses.
     */
    static void PrintAvailableBusses() {
        MELO_INFO_STREAM("Available adapters:");
        ec_adaptert* adapter = ec_find_adapters();
        while (adapter != nullptr) {
            MELO_INFO_STREAM("- Name: '" << adapter->name << "', description: '" << adapter->desc << "'");
            adapter = adapter->next;
        }
    }

    /*!
     * In-place construction of a new slave.
     * @param options Pointer to the option class of the slave.
     * @return True if successful.
     */
    template <class C, typename TOptions>
    inline std::pair<C*, bool> addSlave(TOptions* options) {
        C* dev = new C(options);
        bool success = addDevice(dev);
        return std::make_pair(dev, success);
    }

    /*!
     * Adds a slave to the device vector and calls its initDevice function
     * @param slave Pointer to the slave.
     * @return True if successful.
     */
    inline bool addSlave(EtherCatSlave* slave) {
        // assign the slave some id to calculate the offset in ethernet frame address
        slaves_.push_back(slave);
        return slave->initDeviceInternal(this);
    }

    /*!
     * Add a TxPDO callback method. Every slave can only register one callback method.
     * @param slave    Slave to call method from.
     * @param function Pointer to the slave's method.
     * @return True if successful.
     */
    template <class T>
    inline bool addTxPdoCallback(T* slave, bool(std::common_type<T>::type::*function)(const EtherCatDatagram&)) {
        return txPdoCallbackMap_.emplace(slave, std::bind(function, slave, std::placeholders::_1)).second;
    }

    /*!
     * Setup the bus communication.
     * @param maxRetries Maximal number of retries if not successful.
     * @param retrySleep Time to sleep between retries in seconds.
     * @return True if successful.
     */
    bool setupCommunication(const unsigned int maxRetries = 0, const double retrySleep = 1.0) {
        // Initialize SOEM.
        for (unsigned int retry = 0; retry <= maxRetries; retry++) {
            if (ecx_config_init(&ecatContext_, FALSE) > 0) {
                // Successful initialization.
                break;
            } else if (retry == maxRetries) {
                // Too many failed attempts.
                MELO_ERROR_STREAM("Bus '" << options_->name_ << "': No slaves have been found.");
                return false;
            }
            // Sleep and retry.
            threadSleep(retrySleep);
            MELO_INFO_STREAM("Bus '" << options_->name_ << "': No slaves have been found, retrying " << retry+1 << "/" << maxRetries << " ...");
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

        // Enable/disable symmetrical transfers.
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
        MELO_INFO_STREAM("Bus '" << options_->name_ << "': Calculated expected working counter: " << wkcExpected_.load());

        communicationIsSetUp_ = true;

/*
        // Go to state Safe Op.
        for (EtherCatSlave* slave : slaves_) {
            setStateSafeOp(slave->getAddress());
            if (!waitForStateSafeOp(slave->getAddress())) {
                MELO_ERROR_STREAM("Bus '" << options_->name_ << "': Slave " << slave->getAddress() << " did not reach the safe-op state.");
                return false;
            }
            MELO_INFO_STREAM("      actual state: " << ecatContext_.slavelist[slave->getAddress()].state);
        }
*/
        // Go to state Operational.
        for (EtherCatSlave* slave : slaves_) {
            setStateOperational(slave->getAddress());
            if (!waitForStateOperational(slave->getAddress(), 50, 0.002)) {
                MELO_ERROR_STREAM("Bus '" << options_->name_ << "': Slave " << slave->getAddress() << " did not reach the operational state.");
                MELO_ERROR_STREAM("      actual state: " << ecatContext_.slavelist[slave->getAddress()].state);
                MELO_ERROR_STREAM("      actual status code: " << ecatContext_.slavelist[slave->getAddress()].ALstatuscode );
                return false;
            }
        }


        // Print slave errors.
        printErrorReport();

        return true;
    }

    /*!
     * Set the desired EtherCAT state machine state to Init.
     * @param slave Address of the slave, 0 means all slaves.
     */
    void setStateInit(const uint16_t slave = 0) {
        setState(EC_STATE_INIT, slave);
    }

    /*!
     * Set the desired EtherCAT state machine state to PreOp.
     * @param slave Address of the slave, 0 means all slaves.
     */
    void setStatePreOp(const uint16_t slave = 0) {
        setState(EC_STATE_PRE_OP, slave);
    }

    /*!
     * Set the desired EtherCAT state machine state to Boot.
     * @param slave Address of the slave, 0 means all slaves.
     */
    void setStateBoot(const uint16_t slave = 0) {
        setState(EC_STATE_BOOT, slave);
    }

    /*!
     * Set the desired EtherCAT state machine state to SafeOp.
     * @param slave Address of the slave, 0 means all slaves.
     */
    void setStateSafeOp(const uint16_t slave = 0) {
        setState(EC_STATE_SAFE_OP, slave);
    }

    /*!
     * Set the desired EtherCAT state machine state to Operational.
     * @param slave Address of the slave, 0 means all slaves.
     */
    void setStateOperational(const uint16_t slave = 0) {
        setState(EC_STATE_OPERATIONAL, slave);
    }

    /*!
     * Wait for the EtherCAT state machine state Init to be reached.
     * @param slave      Address of the slave, 0 means all slaves.
     * @param maxRetries Maximum number of retries.
     * @param retrySleep Duration to sleep between the retries.
     * @return True if the state has been reached within the timeout.
     */
    bool waitForStateInit(
        const uint16_t slave = 0,
        const unsigned int maxRetries = maxRetriesDef_,
        const double retrySleep = retrySleepDef_) {
        return waitForState(EC_STATE_INIT, slave, maxRetries, retrySleep);
    }

    /*!
     * Wait for the EtherCAT state machine state PreOp to be reached.
     * @param slave      Address of the slave, 0 means all slaves.
     * @param maxRetries Maximum number of retries.
     * @param retrySleep Duration to sleep between the retries.
     * @return True if the state has been reached within the timeout.
     */
    bool waitForStatePreOp(
        const uint16_t slave = 0,
        const unsigned int maxRetries = maxRetriesDef_,
        const double retrySleep = retrySleepDef_) {
        return waitForState(EC_STATE_PRE_OP, slave, maxRetries, retrySleep);
    }

    /*!
     * Wait for the EtherCAT state machine state Boot to be reached.
     * @param slave      Address of the slave, 0 means all slaves.
     * @param maxRetries Maximum number of retries.
     * @param retrySleep Duration to sleep between the retries.
     * @return True if the state has been reached within the timeout.
     */
    bool waitForStateBoot(
        const uint16_t slave = 0,
        const unsigned int maxRetries = maxRetriesDef_,
        const double retrySleep = retrySleepDef_) {
        return waitForState(EC_STATE_BOOT, slave, maxRetries, retrySleep);
    }

    /*!
     * Wait for the EtherCAT state machine state SafeOp to be reached.
     * @param slave      Address of the slave, 0 means all slaves.
     * @param maxRetries Maximum number of retries.
     * @param retrySleep Duration to sleep between the retries.
     * @return True if the state has been reached within the timeout.
     */
    bool waitForStateSafeOp(
        const uint16_t slave = 0,
        const unsigned int maxRetries = maxRetriesDef_,
        const double retrySleep = retrySleepDef_) {
        return waitForState(EC_STATE_SAFE_OP, slave, maxRetries, retrySleep);
    }

    /*!
     * Wait for the EtherCAT state machine state Operational to be reached.
     * @param slave      Address of the slave, 0 means all slaves.
     * @param maxRetries Maximum number of retries.
     * @param retrySleep Duration to sleep between the retries.
     * @return True if the state has been reached within the timeout.
     */
    bool waitForStateOperational(
        const uint16_t slave = 0,
        const unsigned int maxRetries = maxRetriesDef_,
        const double retrySleep = retrySleepDef_) {
        return waitForState(EC_STATE_OPERATIONAL, slave, maxRetries, retrySleep);
    }

    /*!
     * Synchronize the distributed clocks.
     * TODO Specify the state the slaves have to be in.
     * @param slave    Address of the slave.
     * @param activate True to activate, false to deactivate.
     */
    void syncDistributedClocks(const uint16_t slave, const bool activate) {
        MELO_INFO_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": " << (activate ? "Activating" : "Deactivating") << " distributed clock synchronization ...")
        const double timeStep = 1e-3;
        ecx_dcsync0(&ecatContext_, slave, static_cast<boolean>(activate), static_cast<uint32>(timeStep*1e9), static_cast<int32>(0.5*1e9*timeStep));
        MELO_INFO_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": Finished " << (activate ? "activating" : "deactivating") << " distributed clock synchronization.")
    }

    /*!
     * Print the state of the distributed clocks.
     */
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

    /*!
     * Send a writing SDO.
     * @param slave          Address of the slave.
     * @param index          Index of the SDO.
     * @param subindex       Sub-index of the SDO.
     * @param completeAccess Access all sub-inidices at once.
     * @param value          Value to write.
     * @return True if successful.
     */
    template <typename Value>
    bool sendSdoWrite(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value) {
        //MELO_INFO("sendSdoWrite");
        const int size = sizeof(Value);
        Value valueCopy = value; // copy value to make it modifiable
        const int wkc = ecx_SDOwrite(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), size, &valueCopy, EC_TIMEOUTRXM);
        //MELO_INFO("sendSdoWrite done");
        if (wkc <= 0) {
            MELO_ERROR_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": Working counter too low ("
                << wkc << ") for writing SDO 0x" << std::hex << index <<  ".0x" << std::hex << static_cast<uint16_t>(subindex));
            return false;
        }
        return true;
    }

    /*!
     * Send a reading SDO.
     * @param slave          Address of the slave.
     * @param index          Index of the SDO.
     * @param subindex       Sub-index of the SDO.
     * @param completeAccess Access all sub-inidices at once.
     * @param value          Return argument, will contain the value which was read.
     * @return True if successful.
     */
    template <typename Value>
    bool sendSdoRead(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value) {
        //MELO_INFO("sendSdoRead");
        int size = sizeof(Value);
        const int wkc = ecx_SDOread(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), &size, &value, EC_TIMEOUTRXM);
        //MELO_INFO("sendSdoRead done");
        if (wkc <= 0) {
            MELO_ERROR_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": Working counter too low ("
                << wkc << ") for reading SDO 0x" << std::hex << index <<  ".0x" << std::hex << static_cast<uint16_t>(subindex));
            return false;
        }
        if (size != sizeof(Value)) {
            MELO_ERROR_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": Size mismatch (expected " << sizeof(Value) << " bytes, read "
                << size << " bytes) for reading SDO 0x" << std::hex << index <<  ".0x" << std::hex << static_cast<uint16_t>(subindex));
            return false;
        }
        return true;
    }

    /*!
     * Send a message to a slave.
     * This method stages a pair of Rx and Px Datagrams for one slave.
     * As soon as a pair for every slave has been staged, the data is actually sent.
     * @param slave          Address of the slave.
     * @param rxAndTxPdoDatagram Pair of Rx and Tx PDO datagrams.
     */
    void sendMessage(const uint16_t slave, const std::pair<EtherCatDatagram, EtherCatDatagram>& rxAndTxPdoDatagram) {
        // Create a new staged datagrams object if not existing yet.
        if (!stagedDatagrams_) {
            stagedDatagrams_.reset(new tcan_ethercat::EtherCatDatagrams());
        }

        // Stage the Rx and Tx PDO datagrams for one slave.
        stagedDatagrams_->rxAndTxPdoDatagrams_.insert({slave, rxAndTxPdoDatagram});

        // Only send the message once the Rx and Tx datagrams of all slaves have been staged.
        if (static_cast<int>(stagedDatagrams_->rxAndTxPdoDatagrams_.size()) < ecatSlavecount_) {
            return;
        }

        // Send all datagrams at once and clear the staged datagrams.
        tcan::Bus<EtherCatDatagrams>::sendMessage(*stagedDatagrams_); // TODO: Use return bool.
        stagedDatagrams_.reset();
    }

    // TODO: Implement emplace method.
//    void emplaceMessage(const uint16_t slave, const std::pair<EtherCatDatagram, EtherCatDatagram>& rxAndTxPdoDatagram) {}

    /*!
     * Get the last EtherCAT datagrams which were received.
     * @return Last received EtherCAT datagrams.
     */
    std::shared_ptr<EtherCatDatagrams> getData() {
        return receivedDatagrams_;
    }

 protected:
    /*!
     * Sleep.
     * @param duration Sleeping duration in seconds.
     */
    void threadSleep(const double duration) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int>(1e9*duration)));
    }

    /*!
     * Initialize the interface.
     * @return true if successful
     */
    bool initializeInterface() {
        /*
         * Followed by start of the application we need to set up the NIC to be used as
         * EtherCAT Ethernet interface. In a simple setup we call ec_init(ifname) and if
         * SOEM comes with support for cable redundancy we call ec_init_redundant that
         * will open a second port as backup. You can send NULL as ifname if you have a
         * dedicated NIC selected in the nicdrv.c. It returns >0 if succeeded.
         */
        if (!busIsAvailable()) {
            MELO_ERROR_STREAM("Bus '" << options_->name_ << "': Bus is not available.");
            PrintAvailableBusses();
            return false;
        }
        if (ecx_init(&ecatContext_, options_->name_.c_str()) <= 0) {
            MELO_ERROR_STREAM("Bus '" << options_->name_ << "': No socket connection. Execute as root.");
            return false;
        }
        busIsOpen_ = true;

        MELO_INFO_STREAM("Bus '" << options_->name_ << "': EtherCAT initialization succeeded.");

        return true;
    }

    /*!
     * Cleanup the interface.
     */
    void cleanupInterface() {
        if (*ecatContext_.slavecount > 0) {
            // Set the slaves to state Init.
            setStateInit();
            waitForStateInit();
        }

        // Close the port.
        if (busIsOpen_ && ecatContext_.port) {
            MELO_INFO_STREAM("Bus '" << options_->name_ << "': Closing socket ...");
            ecx_close(&ecatContext_);
            sleep(0.5); // Sleep to make sure the socket is closed, because ecx_close is non-blocking.
        }

        // Delete all slaves.
        if (slaves_.size() > 0) {
            MELO_INFO_STREAM("Bus '" << options_->name_ << "': Deleting slaves ...");
            for (auto slave : slaves_) {
                delete slave;
            }
        }
    }

    /*!
     * Is called after reception of a message. Routes the message to the callback.
     * @param msg Reference to the EtherCAT datagrams.
     */
    void handleMessage(const EtherCatDatagrams& msg) {
        // TODO:
        // Check time the frame took to travel through the physical bus
        for (EtherCatSlave* slave : slaves_) {
            slave->resetDeviceTimeoutCounter();
            auto callback = txPdoCallbackMap_.find(slave);
            if (callback != txPdoCallbackMap_.end()) {
                callback->second(msg.rxAndTxPdoDatagrams_.at(slave->getAddress()).second); //TODO improve access
            }
        }
    }

    /*!
     * Send EtherCAT process data.
     */
    void sendProcessData() {
        if (!communicationIsSetUp_) {
            return;
        }
        ecx_send_processdata(&ecatContext_);
    }

    /*!
     * Receive EtherCAT process data and update the working counter.
     */
    void receiveProcessData() {
        if (!communicationIsSetUp_) {
            return;
        }
        wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
    }

    /*!
     * Write datagrams to the device driver.
     * @return True the data has been written successfully.
     */
    bool writeData(std::unique_lock<std::mutex>* lock) override {
        // Copy the datagrams to send to the sent datagrams.
        sentDatagrams_.reset(new EtherCatDatagrams(outgoingMsgs_.front()));
        if (lock != nullptr) {
            lock->unlock();
        }

        // Copy the Rx PDO datagram payloads from the outgoing message to SOEM.
        for (const auto& rxAndTxDatagram : sentDatagrams_->rxAndTxPdoDatagrams_) {
            memcpy(
                ecatContext_.slavelist[rxAndTxDatagram.second.first.header_.address_].outputs,
                rxAndTxDatagram.second.first.getData(),
                rxAndTxDatagram.second.first.getDataLength());
        }

        // Send the process data in SOEM.
        sendProcessData();

        if(lock != nullptr) {
            lock->lock();
        }
        outgoingMsgs_.pop_front();

        return true;
    }

    /*!
     * Read datagrams from the device driver.
     * @return True the data has been read and parsed successfully.
     */
    bool readData() override {
        // readData() is called as long as it returns successful.
        // Therefore is has to return false as soon as there is nothing to read anymore.
        if (!sentDatagrams_) {
            return false;
        }

        // Receive the process data from SOEM.
        receiveProcessData();

        // Check if the working counter is fine.
        if (!workingCounterIsOk()) {
            MELO_WARN_STREAM("Bus '" << options_->name_ << "': Working counter is too low (" << wkc_ << " < " << wkcExpected_ << ").");
            return false;
        }

        receivedDatagrams_.reset(new EtherCatDatagrams(*sentDatagrams_));
        sentDatagrams_.reset();

        // Copy the Tx PDO datagram payloads from SOEM to the send datagrams.
        for (int i = 1; i <= *ecatContext_.slavecount; i++) {
            memcpy(
                receivedDatagrams_->rxAndTxPdoDatagrams_[i].second.data_,
                ecatContext_.slavelist[i].inputs,
                receivedDatagrams_->rxAndTxPdoDatagrams_[i].second.getDataLength());
        }

        // Handle the new message.
        handleMessage(*receivedDatagrams_);

        return true;
    }

    /*!
     * Do a sanity check of all slaves on this bus.
     */
    bool sanityCheck() override {
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
        bool allMissing = true;
        for (auto slave : slaves_) {
            isMissingOrError |= !slave->sanityCheck();
            allActive &= slave->isActive();
            allMissing &= slave->isMissing();
        }

        isMissingDeviceOrHasError_ = isMissingOrError;
        allDevicesActive_ = allActive;
        allDevicesMissing_ = allMissing;

        return !(isMissingOrError || hasBusError_);
    }

    /*!
     * Check if the expected and detected slaves match.
     * @return True if matching.
     */
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

    /*!
     * Print slave info.
     */
    void printSlaveInfo() {
        int ret = 0;
        int Osize = 0, Isize = 0;

        const unsigned int nsdo = 16;
        int sdodata[nsdo], sdodatasize;
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
        for (uint16 k = 0; k < odinfo.Entries; k++) {
            ecx_readODdescription(&ecatContext_, k, &odinfo);
            ecx_readOE(&ecatContext_, k, &odinfo, &odentryinfo);
            MELO_INFO_STREAM("Index = 0x" << std::hex << odinfo.Index[k]);
            MELO_INFO_STREAM("    MaxSub     = " << odinfo.MaxSub[k]+1);
            MELO_INFO_STREAM("    ObjectCode = " << odinfo.ObjectCode[k]);
            MELO_INFO_STREAM("    DataType   = " << odinfo.DataType[k]);
            MELO_INFO_STREAM("    Description: " << &odinfo.Name[k][0]);
            MELO_INFO_STREAM("    OE Entries = " << odentryinfo.Entries);
            for (uint16 j = 0; j < odentryinfo.Entries; j++) {
                for (unsigned int n = 0; n < nsdo; n++) {
                    sdodata[n] = 0;
                }
                sdodatasize = nsdo*sizeof(int);
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

    /*!
     * Print an error report.
     */
    void printErrorReport() {
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

    /*!
     * Print all IO segments.
     */
    void printIoSegments() {
        MELO_INFO_STREAM("Bus '" << options_->name_ << "': IO Segments: " <<
                ecatContext_.grouplist[0].nsegments << ": " <<
                ecatContext_.grouplist[0].IOsegment[0] << ", " <<
                ecatContext_.grouplist[0].IOsegment[1] << ", " <<
                ecatContext_.grouplist[0].IOsegment[2] << ", " <<
                ecatContext_.grouplist[0].IOsegment[3]);
    }

    /*!
     * Check if the working counter is big enough.
     * @return True if the working counter is equal or higher than expected.
     */
    bool workingCounterIsOk() {
        return wkc_ >= wkcExpected_;
    }

    /*!
     * Set the desired EtherCAT state machine state.
     * @param state Desired state.
     * @param slave Address of the slave, 0 means all slaves.
     */
    void setState(const uint16_t state, const uint16_t slave = 0) {
        ecatContext_.slavelist[slave].state = state;
        ecx_writestate(&ecatContext_, slave);
        MELO_INFO_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": State " << state << " has been set.");
    }

    /*!
     * Wait for an EtherCAT state machine state to be reached.
     * @param state      Desired state.
     * @param slave      Address of the slave, 0 means all slaves.
     * @param maxRetries Maximum number of retries.
     * @param retrySleep Duration to sleep between the retries.
     * @return True if the state has been reached within the timeout.
     */
    bool waitForState(
        const uint16_t state,
        const uint16_t slave = 0,
        const unsigned int maxRetries = maxRetriesDef_,
        const double retrySleep = retrySleepDef_) {
        for (unsigned int retry = 0; retry <= maxRetries; retry++) {
            if (ecx_statecheck(&ecatContext_, slave, state, static_cast<int>(1e6*retrySleep)) == state) {
                MELO_INFO_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": State " << state << " has been reached.");
                return true;
            }
            sendProcessData();
            receiveProcessData();
        }

        MELO_WARN_STREAM("Bus '" << options_->name_ << "', slave " << slave << ": State " << state << " has not been reached.");
        return false;
    }

protected:
    // Default value for the maximal retries to wait for a state.
    static constexpr unsigned int maxRetriesDef_ = 40;
    // Default value for the duration to sleep between the retries.
    static constexpr double retrySleepDef_ = 0.001;

    // Vector containing all slaves.
    std::vector<EtherCatSlave*> slaves_;

    // Map mapping COB id to parse functions.
    TxPdoCallbackMap txPdoCallbackMap_;

    // Datagrams staged for sending.
    std::shared_ptr<EtherCatDatagrams> stagedDatagrams_;
    // Datagrams which have been sent.
    std::shared_ptr<EtherCatDatagrams> sentDatagrams_;
    // Datagrams which have been received.
    std::shared_ptr<EtherCatDatagrams> receivedDatagrams_;

    // EtherCAT input/output mapping of the slaves within the datagrams.
    char ioMap_[4096];

    // Working counters.
    std::atomic<int> wkcExpected_;
    std::atomic<int> wkc_;

    // Bool indicating whether the bus has been opened.
    bool busIsOpen_ = false;
    // Bool indicating whether the communication has been set up.
    bool communicationIsSetUp_ = false;

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
    // Note: SOEM does not use dynamic memory allocation (new/delete). Therefore
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
