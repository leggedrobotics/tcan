/*!
 * @file 	SDOManger.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#ifndef SDOMANAGER_HPP_
#define SDOMANAGER_HPP_


#include "libcanplusplus/SDOMsg.hpp"
#include <list>


//! Service Data Object (SDO) Manager
/*!
 * @ingroup robotCAN
 */
class SDOManager {
public:
	/*! Constructor
	 * @param iBus	identifier of the CAN bus (channel)
	 */
	SDOManager(int iBus);

	//! Destructor
	virtual ~SDOManager();

	/*! Adds an SDO message to the list
	 * @param sdo 	reference to the SDO
	 */
	virtual void addSDO(SDOMsg* sdo);

	/*! Adds an SDO message to the list
	 *	Use this function if you have a pointer to the SDO
	 * @param sdo reference to the SDO
	 */
	virtual void addSDO(SDOMsgPtr sdo);

	/*! Gets the reference to a SDO by index
	 * @param 	index		index of the SDO in the list
	 * @return 	reference to SDO
	 */
	virtual SDOMsg* getSDO(unsigned int index);

	/*! Gets the number of SDOs in the list
	 * @return number of SDOs
	 */
	virtual int getSize();

	/*! test is we are still waiting to send or receive some SDO
	 */
    bool isEmpty();

	/*! Gets the reference to a SDO by index
	 * @param 	index		index of the SDO in the list
	 * @return 	reference to SDO
	 */
	virtual SDOMsg* operator[] (unsigned int index){return getSDO(index);}

	/*! Gets the first SDO in the list
	 * @return reference to SDO
	 */
	virtual SDOMsg* getFirstSDO();

	/*! Gets the SDO that is sent
	 *
	 * @return reference to SDO
	 */
	virtual SDOMsg* getSendSDO();

	/*!  Gets the SDO that is received
	 *
	 * @return reference to SDO
	 */
	virtual SDOMsg* getReceiveSDO();

protected:
	//! List of SDO messages that works as a buffer
	std::list<SDOMsgPtr>sdos_;

	//! An empty SDO message
	SDOMsg* emptySDO_;

	//! identifier of the CAN bus (channel)
	int iBus_;
};

#endif /* SDOMANAGER_HPP_ */
