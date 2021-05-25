#pragma once

#include <thread>

namespace tcan {

bool setThreadPriority(std::thread& thread, const int priority);
bool raiseThreadPriority(std::thread& thread, const int priority);

inline int calculatePollTimeoutMs(const timeval& tv) {
    // normal infinity timeout is specified with timeout of 0. poll has infinity for negative values, so subtract 1ms
    return (tv.tv_sec*1000 + tv.tv_usec/1000)-1;
}

};