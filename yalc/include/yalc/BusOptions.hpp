/*!
 * @file 	Bus.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#ifndef BUSOPTIONS_HPP_
#define BUSOPTIONS_HPP_

class BusOptions {
public:

	BusOptions():
		asynchronous(true),
		sanityCheckInterval(1000),
		priorityReceiveThread(99),
		priorityTransmitThread(98),
		prioritySanityCheckThread(1)
	{

	}

	BusOptions(const bool async, const unsigned int sanity_check_interval):
		asynchronous(async),
		sanityCheckInterval(sanity_check_interval),
		priorityReceiveThread(99),
		priorityTransmitThread(98),
		prioritySanityCheckThread(1)
	{

	}

	virtual ~BusOptions() { }

	//! will create recieve and transmit threads if set to true
	bool asynchronous;

	//! if > 0, a thread will be created which does a sanity check of the devices. [ms]
	unsigned int sanityCheckInterval;

	int priorityReceiveThread;
	int priorityTransmitThread;
	int prioritySanityCheckThread;


};

#endif /* BUSOPTIONS_HPP_ */
