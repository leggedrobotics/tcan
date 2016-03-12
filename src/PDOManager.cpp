/*!
 * @file 	PDOManger.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#include "libcanplusplus/PDOManager.hpp"

#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <iostream>

PDOManager::PDOManager():isSending_(false)
{

}
PDOManager::~PDOManager()
{
	pdos_.clear();
}

void PDOManager::addPDO(CANOpenMsg* pdo)
{
	pdos_.push_back(pdo);
}

int PDOManager::getSize()
{
	return pdos_.size();
}

CANOpenMsg* PDOManager::getPDOWithCOBId(unsigned int id) {
    for (unsigned int i=0;i<pdos_.size();i++) {
        if (pdos_[i].getCOBId() == (signed)id) {
            return &(pdos_[i]);
        }
    }
    return NULL;
}

CANOpenMsg* PDOManager::getPDO(unsigned int index)
{
	try {
		if (index >= pdos_.size()) {
			std::string error = "PDOManager: Could not get PDO with index "
								+ boost::lexical_cast<std::string>(index) + "!";
			throw std::out_of_range(error);
		}
		return &(pdos_[index]);

	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
	}
	return NULL;
}

void PDOManager::setSending(bool isSending)
{
	isSending_ = isSending;
}
bool PDOManager::isSending()
{
	return isSending_;
}
