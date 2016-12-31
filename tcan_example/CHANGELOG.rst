^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tcan_example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2016-12-30)
------------------
* removed robot_utils dependency
* fixed wronk IP interface name
* implemented tcp connections
* sleeping before first executing first sanity check (to prevent checking an empty device list)
* implemented binding of message callbacks for non-Device based classes
* working example with usb
* compilable, implemented structure for USB
* fixed tcan_example
* renamed robotUtils to robot_utils (resolves install space issues)
* added TxEmcyId
* fixed wrong handleReadSdoAnswer function definition in example
* added addDevice function with options as parameter
* Switched from m545_utils to robot_utils for multikey container
* added new write functions to CanMsg, cleanups
* Improved error printouts, implemented maxQueueSize and resolved issue #5 (sending sdo after reception of any heartbeat)
* renamed to tcan
* Contributors: Philipp Leemann, Remo Diethelm
