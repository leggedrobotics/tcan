^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tcan
^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* ignoring arbitration lost errors
* added functions to check for bus errors
* added parsing of bus error frames
* improved comments
* added candump parsing python script
* added enum for specifying TCP or UDP (to be implemented)
* fixed wronk IP interface name
* implemented tcp connections
* renamed UsbMsg to GenericMsg, typedef UsbMsg to this new class
* sleeping before first executing first sanity check (to prevent checking an empty device list)
* added hasError to CanDevice
* fixed resetDeviceTimeoutCounter
* added scripts to set up can interface, removed return values from sanityCheck()
* added error state, raised by SDO errors
* comment cleaned
* restoring serial settings after shutdown, uint8_t instead of chars to store data, switching non-canonical and canonical modes
* implemented timeout for USB
* fixed merge bugs
* fixed wrong template deduction
* implemented binding of message callbacks for non-Device based classes
* removed addCanMessage with std::bind, renamed checkDeviceTimeout function
* added handleSdoError(..) callback
* restructured code
* fixed wrong return value of BusManager::isMissingDevice(), returning a.s.a.p
* implemented simple state machine for Device class, moved configureDevice from DeviceCanOpen to Device
* implemented BusManager::allBusesOperational()
* working example with usb
* compiling usb example
* compilable, implemented structure for USB
* (wip)
* declared sendPdo and sendSdo as public
* added helper functions to calculate timeout counters
* added comments
* added addCanMessage(.., CalbackPtr&& ..) to allow callback functions with class members which are not derived from tcan::Device
* fixed template deduction error
* also saving device pointer in callback pointer map, resetting device timout on reception of every message
* cleanup
* resetting device timeout in DeviceCanOpen::setNmtRestartRemoteDevice and sdo timeout counters in DeviceCanOpen::sendSdo
* reverted resetting sdoSentCounter on reception of sdo
* added mutex for sdoAnswerMap
* reseting sdoSentCounter on reception of sdo
* Erase previous sdo answer if request is sent again; class DeviceCanOpenOptions is now similar to DeviceOptions
* added changelogs
* fixed missing include
* fixed read SDO answer bug, ignoring wrong message length of heartbeat messages (temporary)
* added convenience function to send pdo
* added TxEmcyId
* fixed indentation
* using unordered SDO answer map, overwrite entries
* added SDO answer map
* added addDevice function with options as parameter
* stoping all threads before joining them, shorter shutdown
* moved mutex lock in processOutputQueue()
* fixed wrong error printout
* Fixed read-sdo dynamic_cast
* Used 'convenient' hedaer file from  message_logger
* Added message_logger dependency
* Fixed overwriting of nmt states set in configureDevice(..)
* Added BusManager::sendSync(..) to send sync on single bus
* resetting DeviceTimout on SDO reception, fixed sdo queue bug
* moved configureDevice() drom Device to DeviceCanOpen and also call it if receiving a heartbeat in nmt-state 'missing'
* fixed eclipse indent errors...
* changed indentation using spaces
* BusManager::read/writeMessagesSynchronous does not try to read/write asynchronous buses
* fixed deviceTimout bugs, set default timeout to 2sec
* Fixed sdo COB-Id bug
* Implemented SdoMsg::getCommandByte() and made default constructor accessible
* Changed handleReadSdoAnswer(..) interface
* added new write functions to CanMsg, cleanups
* Improved error printouts, implemented maxQueueSize and resolved issue #5 (sending sdo after reception of any heartbeat)
* renamed to tcan
* Contributors: Remo Diethelm, pleemann
