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

	typedef std::unique_ptr<Bus> BusPtr;

	BusManager();

	virtual ~BusManager();

    bool addBus(const std::string& device);

    virtual bool initializeBus(const std::string& device) = 0;
    virtual bool readMessages() = 0;

	/*! Gets the number of buses
	 * @return	number of buses
	 */
	int getSize() const;

	/*! Gets a reference to a bus by index
	 * @param	index of bus
	 * @return	reference to bus
	 */
    Bus* getBus(const int socketId);


protected:
    std::vector<BusPtr> buses_;
};

#endif /* BUSMANAGER_HPP_ */
