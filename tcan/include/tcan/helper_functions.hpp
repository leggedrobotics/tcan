/*
 * helper_functions.hpp
 *
 *  Created on: May 19, 2017
 *      Author: Philipp Leemann
 */

#pragma once

#include <thread>

namespace tcan {

bool setThreadPriority(std::thread& thread, const int priority);
bool raiseThreadPriority(std::thread& thread, const int priority);

};