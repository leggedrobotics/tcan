#ifndef LOCKED_BUS_MANAGER_H
#define LOCKED_BUS_MANAGER_H

#include <pthread.h>
#include "libcanplusplus/BusManager.hpp"

class LockedBusManager : public BusManager
{
	public:
		LockedBusManager() : BusManager() {
			pthread_mutex_init(&mutex,NULL);
		}

		void lock() {
			pthread_mutex_lock(&mutex);
		}
		void unlock() {
			pthread_mutex_unlock(&mutex);
		}
	protected:
		pthread_mutex_t mutex;
};


#endif // LOCKED_BUS_MANAGER_H
