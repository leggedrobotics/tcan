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
        BusOptions(name)
    {
    }

    virtual ~CanBusOptions()
    {
    }
};

} /* namespace tcan */
