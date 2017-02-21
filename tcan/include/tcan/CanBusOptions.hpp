/*
 * BusOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/BusOptions.hpp"

namespace tcan {

struct CanBusOptions : public BusOptions {
    CanBusOptions():
        CanBusOptions(std::string())
    {
    }

    CanBusOptions(const std::string& name):
        BusOptions(name),
        passivateOnBusError_(false),
        autoClearBusError_(false)
    {
    }

    virtual ~CanBusOptions()
    {
    }

    //! If set to true, no message are sent on the bus if the bus error flag is set
    bool passivateOnBusError_;

    //! set to true if busErrorFlag_ should be automatically reset on reception of new messages
    bool autoClearBusError_;

};

} /* namespace tcan */
