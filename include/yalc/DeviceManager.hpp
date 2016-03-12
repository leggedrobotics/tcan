/*!
 * @file 	DeviceManager.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */
#ifndef DEVICEMANAGER_HPP_
#define DEVICEMANAGER_HPP_

#include "Bus.hpp"
#include "Device.hpp"
#include <boost/ptr_container/ptr_vector.hpp>

class Bus;
class Device;

//! Device Manager
/*! It manages all devices that are connected by the same CAN bus.
 * @ingroup robotCAN, device
 */
class DeviceManager{
public:
	/*! Constructor
	 * @param bus	reference to the CAN bus the devices are connected to
	 */
	DeviceManager(Bus* bus);

	//! Constructor
	virtual ~DeviceManager();

	/*! Gets the number of devices that are managed
	 * @return number of devices
	 */
	int getSize();

	/*! Adds a new device to the manager
	 *	It sets the reference to the bus of the device
	 *	and adds the PDOs of the device to the PDO managers
	 * @param device
	 */
	void addDevice(Device* device);

	/*! Gets a reference to a device by index
	 * @param index
	 * @return
	 */
	Device* getDevice(unsigned int index);


private:
	//! list of devices
	boost::ptr_vector<Device >devices_;
	//! reference to the bus
	Bus* bus_;
};



#endif /* DEVICEMANAGER_HPP_ */
