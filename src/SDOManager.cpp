/*!
 * @file 	SDOManger.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */


#include "libcanplusplus/SDOManager.hpp"

#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <stdio.h>

SDOManager::SDOManager(int iBus):iBus_(iBus)
{
	emptySDO_ = new SDOMsg(-1, -1, 0);
}

SDOManager::~SDOManager()
{
	delete emptySDO_;
	sdos_.clear();
}

void SDOManager::addSDO(SDOMsg* sdo)
{
	sdo->setIsQueuing(true);
	sdos_.push_back(SDOMsgPtr(sdo));
}

void SDOManager::addSDO(SDOMsgPtr sdo)
{
	sdo->setIsQueuing(true);
	sdos_.push_back(sdo);
}

int SDOManager::getSize()
{
	return sdos_.size();
}

SDOMsg* SDOManager::getSDO(unsigned int index)
{
	std::list<SDOMsgPtr>::iterator iterSDOList;
	unsigned int counter = 0;
	try {
		if (index >= sdos_.size()) {
			std::string error = "SDOManager: Could not get SDO with index "
								+ boost::lexical_cast<std::string>(index) + "!";
			throw std::out_of_range(error);
		}

		for (iterSDOList = sdos_.begin(); iterSDOList != sdos_.end(); iterSDOList++) {
		  if (counter == index) {
			  return iterSDOList->get();
		  }
		  counter++;
		}

	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
	}
	return NULL;
}

SDOMsg* SDOManager::getFirstSDO()
{
	try {
		if (getSize() == 0) {
			std::string error = "SDOManager: Could not get first SDO!";
			throw std::out_of_range(error);
		}
		return sdos_.front().get();

	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
	}
	return NULL;
}

SDOMsg* SDOManager::getSendSDO()
{
	if (getSize() == 0) {
		return emptySDO_;
	}
	SDOMsg* firstSDO = getFirstSDO();
	if (firstSDO->getIsSent() && firstSDO->getIsReceived()) {
		/* the first SDO message was setn and received */
		sdos_.pop_front();
	}
	if (getSize() == 0) {
		/* the SDO list is empty */
		return emptySDO_;
	}
	return getFirstSDO();
}

SDOMsg* SDOManager::getReceiveSDO()
{
	if (getSize() == 0) {
		/* the SDO list is empty */
		return emptySDO_;
	}
	if (getFirstSDO()->hasTimeOut()) {
		printf("\e[0;31m\n* * * * * * * * * * * * *\n");
		printf("* SDO problem: no answer received!\n");
		printf("* Bus: %d; COB_ID: %X; index: %02X%02X; subindex: %X\n",
				iBus_,
				getFirstSDO()->getOutputMsg()->getCOBId(),
				getFirstSDO()->getOutputMsg()->getValue()[2],
				getFirstSDO()->getOutputMsg()->getValue()[1],
				getFirstSDO()->getOutputMsg()->getValue()[3]);
		printf("* * * * * * * * * * * * *\n\e[0m");
		sdos_.pop_front();
	}
	if (getSize() == 0) {
		return emptySDO_;
	}
	return getFirstSDO();
}

bool SDOManager::isEmpty() {
    return getSendSDO() == emptySDO_;
}
