/*!
 * @file 	Bus.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#ifndef BUS_HPP_
#define BUS_HPP_

class Bus {
public:
	Bus(int iBus);

	virtual ~Bus();

private:

    unsigned int syncInterval_;
};

#endif /* BUS_HPP_ */
