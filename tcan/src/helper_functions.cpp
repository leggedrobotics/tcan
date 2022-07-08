#include "tcan/helper_functions.hpp"

namespace tcan {

bool setThreadPriority(std::thread& thread, const int priority) {
    sched_param sched;
    sched.sched_priority = priority;
    if (pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &sched) != 0) {
        return false;
    }
    return true;
}

bool raiseThreadPriority(std::thread& thread, const int priority) {
    sched_param sched;
    int policy;
    pthread_getschedparam(thread.native_handle(), &policy, &sched);

    if(policy != SCHED_FIFO || sched.sched_priority < priority) {
        return setThreadPriority(thread, priority);
    }
    return true;
}

} // namespace tcan