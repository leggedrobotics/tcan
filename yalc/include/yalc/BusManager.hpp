/*!
 * @file 	BusManager.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */
#ifndef BUSMANAGER_HPP_
#define BUSMANAGER_HPP_

#include <memory>
#include <thread>
#include <vector>
#include <atomic>
#include <condition_variable>

#include "yalc/Bus.hpp"

//! Container of all CAN buses
/*! Manager to facilitate the handling of various CAN buses.
 * @ingroup robotCAN, bus
 */
class BusManager {
public:

	typedef std::unique_ptr<Bus> BusPtr;

	BusManager();

	virtual ~BusManager();

	bool addBus(const std::string& interface);

	virtual bool initializeBus(const std::string& interface) = 0;
	virtual bool readMessages() = 0;
	virtual bool writeMessages() = 0;

	/*! Gets the number of buses
	 * @return	number of buses
	 */
	unsigned int getSize() const { return buses_.size(); }

	/*! Gets a reference to a bus by index
	 * @param	index of bus
	 * @return	reference to bus
	 */
//    Bus* getBus(const unsigned int index) { return buses_.at(index); }

	void notifyTransmitWorker();

	void receiveWorker();
	void transmitWorker();



protected:
	std::vector<BusPtr> buses_;

	// sync interval to simultaneously publish a SYNC message on all buses
	unsigned int globalSyncInterval_;

	// wheter the busmanager should wait until the Output message queues of all buses are empty before sendign the global SYNC.
	// ensures that the sync messages are immediatly sent at the same time and not just appended to a queue.
	bool syncWaitForEmptyQueue_;

	// threads for message reception and transmission
	std::thread receiveThread_;
	std::thread transmitThread_;
	std::atomic<bool> running_;

	// variables to wake the transmitThread after inserting something to the message output queue
	std::atomic<bool> sendingMessages_;
	std::condition_variable condTransmitThread_;
	std::mutex mutexTransmitThread_;
};

#endif /* BUSMANAGER_HPP_ */
