^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tcan_example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* removed robot_utils dependency
* fixed wronk IP interface name
* implemented tcp connections
* sleeping before first executing first sanity check (to prevent checking an empty device list)
* implemented binding of message callbacks for non-Device based classes
* working example with usb
* compiling usb example
* compilable, implemented structure for USB
* fixed tcan_example
* added changelogs
* renamed robotUtils to robot_utils (resolves install space issues)
* added TxEmcyId
* fixed wrong handleReadSdoAnswer function definition in example
* added addDevice function with options as parameter
* Switched from m545_utils to robot_utils for multikey container
* added new write functions to CanMsg, cleanups
* Improved error printouts, implemented maxQueueSize and resolved issue #5 (sending sdo after reception of any heartbeat)
* renamed to tcan
* Contributors: Remo Diethelm, pleemann
