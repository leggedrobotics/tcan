/*
 * BusOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

namespace yalc {

struct BusOptions {
	BusOptions():
		asynchronous(true),
		sanityCheckInterval(100),
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

	//! if > 0 and in asynchronous mode, a thread will be created which does a sanity check of the devices. Default is 100 [ms].
	unsigned int sanityCheckInterval;

	int priorityReceiveThread;
	int priorityTransmitThread;
	int prioritySanityCheckThread;


};

} /* namespace yalc */
