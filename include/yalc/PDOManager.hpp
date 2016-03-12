/*!
 * @file 	PDOManger.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#ifndef PDOMANAGER_HPP_
#define PDOMANAGER_HPP_

#include <boost/ptr_container/ptr_vector.hpp>
#include "libcanplusplus/CANOpenMsg.hpp"

//! Process Data Object (PDO) Manager
/*!
 * @ingroup robotCAN
 */
class PDOManager {
public:
	//! Constructor
	PDOManager();

	//! Destructor
	virtual ~PDOManager();

	/*! Adds a PDO to the list
	 * Memory deallocation is managed by boost shared ptr.
	 * @param pdo	reference to the PDO
	 */
	void addPDO(CANOpenMsg* pdo);

	/*! Gets the reference to a PDO by index
	 *
	 * @param index	index of the PDO in the list
	 * @return reference to PDO
	 */
	CANOpenMsg*  getPDO(unsigned int index);

	/*! Gets the first PDO with a given COBId
	 *
	 * @param id	desired COBId
	 * @return reference to PDO, NULL if not found
	 */
    CANOpenMsg* getPDOWithCOBId(unsigned int id);

	/*! Short version of getPDO
	 *
	 * @param index index of the PDO in the list
	 * @return reference to PDO
	 */
	CANOpenMsg* operator[] (unsigned int index){return getPDO(index);}

	/*! Gets the number of PDOs in the list
	 * @return number of PDOs
	 */
	int getSize();

	/*! Gets the flag whether the manager is sending PDOs
	 * @param isSending
	 */
	void setSending(bool isSending);

	/*! Gets the flag whether the manager is sending PDOs
	 * @return true if it is sending
	 */
	bool isSending();


private:
	//! list of PDOs that need to be processed
	boost::ptr_vector<CANOpenMsg >pdos_;

	//! true if manage is sending PDOs
	bool isSending_;
};


#endif /* PDOMANAGER_HPP_ */
