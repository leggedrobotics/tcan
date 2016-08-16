^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tcan
^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2016-08-11)
------------------
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
