/*
 * BusManager.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#ifndef BUSMANAGER_HPP_
#define BUSMANAGER_HPP_

#include <memory>
#include <vector>

#include "yalc/Bus.hpp"

namespace yalc {

//! Container of all CAN buses
class BusManager {
public:
	BusManager();

	virtual ~BusManager();

	bool addBus(Bus* bus);

	/*! Gets the number of buses
	 * @return	number of buses
	 */
	unsigned int getSize() const { return buses_.size(); }

	/*! Send a sync message on all buses
	 * @param	wheter the busmanager should wait until the output message queues of all buses are empty before sending the global SYNC.
	 * 			ensures that the sync messages are immediatly sent at the same time and not just appended to a queue.
	 */
	void sendSyncOnAllBuses(const bool waitForEmptyQueues=false);

	void closeBuses();

protected:
	std::vector<Bus*> buses_;

};

} /* namespace yalc */
#endif /* BUSMANAGER_HPP_ */

