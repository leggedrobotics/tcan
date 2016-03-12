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

#include <unordered_map>

#include "libcanplusplus/Bus.hpp"

//! Container of all CAN buses
/*! Manager to facilitate the handling of various CAN buses.
 * @ingroup robotCAN, bus
 */
class BusManager {
public:
	//! Constructor
	BusManager();

	//! Destructor
	virtual ~BusManager();

	/*! Add a new bus by addBus(new Bus(..)).
	 * The deallocation is handled by boost shared pointers.
	 * @param	bus	reference to bus
	 */
	void addBus(Bus* bus);

	/*! Gets the number of buses
	 * @return	number of buses
	 */
	int getSize();

	/*! Gets a reference to a bus by index
	 * @param	index of bus
	 * @return	reference to bus
	 */
	Bus*  getBus(unsigned int index);


private:
    std::map<int, Bus> mapSocketIdToBus_;
};

#endif /* BUSMANAGER_HPP_ */
