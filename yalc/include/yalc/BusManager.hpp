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
#include <vector>

#include "yalc/Bus.hpp"

//! Container of all CAN buses
/*! Manager to facilitate the handling of various CAN buses.
 * @ingroup robotCAN, bus
 */
class BusManager {
public:
	BusManager();

	virtual ~BusManager();

	bool addBus(Bus* bus);

	/*! Gets the number of buses
	 * @return	number of buses
	 */
	unsigned int getSize() const { return buses_.size(); }

	/*! Gets a reference to a bus by index
	 * @param	index of bus
	 * @return	reference to bus
	 */
//    Bus* getBus(const unsigned int index) { return buses_.at(index); }

	void sendSyncOnAllBuses();

protected:
	std::vector<std::unique_ptr<Bus>> buses_;

	// wheter the busmanager should wait until the Output message queues of all buses are empty before sendign the global SYNC.
	// ensures that the sync messages are immediatly sent at the same time and not just appended to a queue.
	bool syncWaitForEmptyQueue_;
};

#endif /* BUSMANAGER_HPP_ */
