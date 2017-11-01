#pragma once

#include "tcan_can/SdoMsg.hpp"

namespace tcan {
namespace canopen {

/** *********************************************************************
----------------------------- Communication -----------------------------
********************************************************************* **/
//////////////////////////////////////////////////////////////////////////////
class SDOSetRS232Baudrate: public SdoMsg
{
public:
  SDOSetRS232Baudrate(const uint32_t nodeId, const uint32_t baudrate):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x2002, 0x00, baudrate)
  {}
  virtual ~SDOSetRS232Baudrate(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetCANBitrate: public SdoMsg
{
public:
  SDOSetCANBitrate(const uint32_t nodeId, const uint32_t bitrate):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x2001, 0x00, bitrate)
  {}
  virtual ~SDOSetCANBitrate(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetAbortConnectionOptionCode: public SdoMsg
{
public:
  SDOSetAbortConnectionOptionCode(const uint32_t nodeId, const uint32_t value):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6007, 0x00, value)
  {}
  virtual ~SDOSetAbortConnectionOptionCode(){}
};


/** *********************************************************************
----------------------------- Initialization ----------------------------
********************************************************************* **/
//////////////////////////////////////////////////////////////////////////////
class SDOControlword: public SdoMsg
{
public:
  SDOControlword(const uint32_t nodeId, const uint32_t controlword):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6040, 0x00, controlword)
  {}
  virtual ~SDOControlword(){}
};
//////////////////////////////////////////////////////////////////////////////
class SDOShutdown: public SdoMsg
{
public:
  SDOShutdown(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6040, 0x00, 0x06)
  {}
  virtual ~SDOShutdown(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOSwitchOn: public SdoMsg
{
public:
  SDOSwitchOn(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6040, 0x00, 0x07)
  {}
  virtual ~SDOSwitchOn(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOEnableOperation: public SdoMsg
{
public:
  SDOEnableOperation(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6040, 0x00, 0x0F)
  {}
  virtual ~SDOEnableOperation(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDODisableOperation: public SdoMsg
{
public:
  SDODisableOperation(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6040, 0x00, 0x07)
  {}
  virtual ~SDODisableOperation(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOFaultReset: public SdoMsg
{
public:
  SDOFaultReset(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6040, 0x00, 0x80)
  {}
  virtual ~SDOFaultReset(){}
};

/************************************************************************
------------------------ Digital Inputs ---------------------------------
********************************************************************* **/
//////////////////////////////////////////////////////////////////////////////
class SDOSetDigitalInputFunctionalitiesMask: public SdoMsg
{
public:
  SDOSetDigitalInputFunctionalitiesMask(const uint32_t nodeId, const uint32_t value):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x2071, 0x02, value)
  {}
  virtual ~SDOSetDigitalInputFunctionalitiesMask(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetDigitalInputFunctionalitiesPolarity
: public SdoMsg
{
public:
  SDOSetDigitalInputFunctionalitiesPolarity(const uint32_t nodeId, const uint32_t value):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x2071, 0x03, value)
  {}
  virtual ~SDOSetDigitalInputFunctionalitiesPolarity(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetDigitalInputFunctionalitiesExecutionMask
: public SdoMsg
{
public:
  SDOSetDigitalInputFunctionalitiesExecutionMask(const uint32_t nodeId, const uint32_t value):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x2071, 0x04, value)
  {}
  virtual ~SDOSetDigitalInputFunctionalitiesExecutionMask(){}
};


/***********************************************************************
-------------------- Life guard and heartbeat control-------------------
********************************************************************* **/
//////////////////////////////////////////////////////////////////////////////
class SDOSetGuardTime: public SdoMsg
{
public:
  SDOSetGuardTime(const uint32_t nodeId, const uint32_t time_ms):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x100C, 0x00, time_ms)
  {}
  virtual ~SDOSetGuardTime(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetLifeTimeFactor: public SdoMsg
{
public:
  SDOSetLifeTimeFactor(const uint32_t nodeId, const uint32_t factor):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x100D, 0x00, factor)
  {}
  virtual ~SDOSetLifeTimeFactor(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOWriteProducerHeartbeatTime: public SdoMsg
{
public:
  SDOWriteProducerHeartbeatTime(const uint32_t nodeId, const uint32_t time_ms):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x1017, 0x00, time_ms)
  {}
  virtual ~SDOWriteProducerHeartbeatTime(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOReadProducerHeartbeatTime: public SdoMsg
{
public:
  SDOReadProducerHeartbeatTime(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::READ, 0x1017, 0x00, 0x0)
  {}

  virtual ~SDOReadProducerHeartbeatTime(){}
};

/***********************************************************************
------------------------------ Utilities --------------------------------
********************************************************************* **/
//////////////////////////////////////////////////////////////////////////////
class SDOSetCOBIDSYNC: public SdoMsg
{
public:
  SDOSetCOBIDSYNC(const uint32_t nodeId, const uint32_t ID):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1005, 0x00, ID)
  {}
  virtual ~SDOSetCOBIDSYNC(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOSaveAllParameters: public SdoMsg
{
public:
  SDOSaveAllParameters(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1010, 0x01, 0x65766173)
  {}
  virtual ~SDOSaveAllParameters(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORestoreAllDefaultParameters: public SdoMsg
{
public:
  SDORestoreAllDefaultParameters(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1011, 0x01, 0x64616F6C)
  {}
  virtual ~SDORestoreAllDefaultParameters(){}
};


/** *********************************************************************
------------------------------ Tx PDO's ---------------------------------
********************************************************************* **/

/*********************************************************************
 * PDO 1 Parameter
 *********************************************************************/

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1Disable: public SdoMsg
{
public:
  SDOTxPDO1Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1800, 0x01, 0x80000180 + nodeId)
  {}
  virtual ~SDOTxPDO1Disable(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1SetInhibitTime: public SdoMsg
{
public:
  SDOTxPDO1SetInhibitTime(const uint32_t nodeId, const uint32_t time_100us):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x1800, 0x03, time_100us)
  {}
  virtual ~SDOTxPDO1SetInhibitTime(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1SetNumberOfMappedApplicationObjects: public SdoMsg
{
public:
  SDOTxPDO1SetNumberOfMappedApplicationObjects(const uint32_t nodeId, const uint32_t number_of_mapped_objects):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1A00, 0x00, number_of_mapped_objects)
  {}
  virtual ~SDOTxPDO1SetNumberOfMappedApplicationObjects(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1ConfigureCOBID: public SdoMsg
{
public:
  SDOTxPDO1ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1800, 0x01, 0x40000180 + nodeId)
  {}
  virtual ~SDOTxPDO1ConfigureCOBID(){}
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1SetTransmissionType: public SdoMsg
{
public:
  SDOTxPDO1SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1800, 0x02, type)
  {}
  virtual ~SDOTxPDO1SetTransmissionType(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1SetMapping: public SdoMsg
{
public:
  SDOTxPDO1SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1A00, indexOfObject, object)
  {}
  virtual ~SDOTxPDO1SetMapping(){}
};


/*********************************************************************
 * PDO 2 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2SetNumberOfMappedApplicationObjects: public SdoMsg
{
public:
  SDOTxPDO2SetNumberOfMappedApplicationObjects(const uint32_t nodeId, const uint32_t number_of_mapped_objects):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1A01, 0x00, number_of_mapped_objects)
  {}
  virtual ~SDOTxPDO2SetNumberOfMappedApplicationObjects(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2ConfigureCOBID: public SdoMsg
{
public:
  SDOTxPDO2ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1801, 0x01, 0x40000280 + nodeId)
  {}
  virtual ~SDOTxPDO2ConfigureCOBID(){}
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2Disable: public SdoMsg
{
public:
  SDOTxPDO2Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1801, 0x01, 0x80000280 + nodeId)
  {}
  virtual ~SDOTxPDO2Disable(){}
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2SetInhibitTime: public SdoMsg
{
public:
    SDOTxPDO2SetInhibitTime(const uint32_t nodeId, const uint32_t time_100us):
            SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x1801, 0x03, time_100us)
    {}
    virtual ~SDOTxPDO2SetInhibitTime(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2SetTransmissionType: public SdoMsg
{
public:
  SDOTxPDO2SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1801, 0x02, type)
  {}
  virtual ~SDOTxPDO2SetTransmissionType(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2SetMapping: public SdoMsg
{
public:
  SDOTxPDO2SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1A01, indexOfObject, object)
  {}
  virtual ~SDOTxPDO2SetMapping(){}
};

/*********************************************************************
 * PDO 3 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3SetNumberOfMappedApplicationObjects: public SdoMsg
{
public:
  SDOTxPDO3SetNumberOfMappedApplicationObjects(const uint32_t nodeId, const uint32_t number_of_mapped_objects):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1A02, 0x00, number_of_mapped_objects)
  {}
  virtual ~SDOTxPDO3SetNumberOfMappedApplicationObjects(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3ConfigureCOBID: public SdoMsg
{
public:
  SDOTxPDO3ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1802, 0x01, 0x40000380 + nodeId)
  {}
  virtual ~SDOTxPDO3ConfigureCOBID(){}
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3Disable: public SdoMsg
{
public:
  SDOTxPDO3Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1802, 0x01, 0x80000380 + nodeId)
  {}
  virtual ~SDOTxPDO3Disable(){}
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3SetInhibitTime: public SdoMsg
{
public:
    SDOTxPDO3SetInhibitTime(const uint32_t nodeId, const uint32_t time_100us):
            SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x1802, 0x03, time_100us)
    {}
    virtual ~SDOTxPDO3SetInhibitTime(){}
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3SetTransmissionType: public SdoMsg
{
public:
  SDOTxPDO3SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1802, 0x02, type)
  {}
  virtual ~SDOTxPDO3SetTransmissionType(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3SetMapping: public SdoMsg
{
public:
  SDOTxPDO3SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1A02, indexOfObject, object)
  {}
  virtual ~SDOTxPDO3SetMapping(){}
};

/*********************************************************************
 * PDO 4 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4SetNumberOfMappedApplicationObjects: public SdoMsg
{
public:
  SDOTxPDO4SetNumberOfMappedApplicationObjects(const uint32_t nodeId, const uint32_t number_of_mapped_objects):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1A03, 0x00, number_of_mapped_objects)
  {}
  virtual ~SDOTxPDO4SetNumberOfMappedApplicationObjects(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4ConfigureCOBID: public SdoMsg
{
public:
  SDOTxPDO4ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1803, 0x01, 0x40000480 + nodeId)
  {}
  virtual ~SDOTxPDO4ConfigureCOBID(){}
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4Disable: public SdoMsg
{
public:
	SDOTxPDO4Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1803, 0x01, 0xFFFFFFFF)
  {}
  virtual ~SDOTxPDO4Disable(){}
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4SetInhibitTime: public SdoMsg
{
public:
    SDOTxPDO4SetInhibitTime(const uint32_t nodeId, const uint32_t time_100us):
            SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x1803, 0x03, time_100us)
    {}
    virtual ~SDOTxPDO4SetInhibitTime(){}
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4SetTransmissionType: public SdoMsg
{
public:
  SDOTxPDO4SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1803, 0x02, type)
  {}
  virtual ~SDOTxPDO4SetTransmissionType(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4SetMapping: public SdoMsg
{
public:
  SDOTxPDO4SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1A03, indexOfObject, object)
  {}
  virtual ~SDOTxPDO4SetMapping(){}
};


/** *********************************************************************
------------------------------ Rx PDO's ---------------------------------
********************************************************************* **/
/*********************************************************************
 * PDO 1 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1SetNumberOfMappedApplicationObjects: public SdoMsg
{
public:
  SDORxPDO1SetNumberOfMappedApplicationObjects(const uint32_t nodeId, const uint32_t number_of_mapped_objects):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1600, 0x00, number_of_mapped_objects)
  {}
  virtual ~SDORxPDO1SetNumberOfMappedApplicationObjects(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1ConfigureCOBID: public SdoMsg
{
public:
  SDORxPDO1ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1400, 0x01, 0x40000200 + nodeId)
  {}
  virtual ~SDORxPDO1ConfigureCOBID(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1Disable: public SdoMsg
{
public:
  SDORxPDO1Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1400, 0x01, 0xFFFFFFFF)
  {}
  virtual ~SDORxPDO1Disable(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1SetTransmissionType: public SdoMsg
{
public:
  SDORxPDO1SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1400, 0x02, type)
  {}
  virtual ~SDORxPDO1SetTransmissionType(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1SetMapping: public SdoMsg
{
public:
  SDORxPDO1SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1600, indexOfObject, object)
  {}
  virtual ~SDORxPDO1SetMapping(){}
};

/*********************************************************************
 * PDO 2 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2SetNumberOfMappedApplicationObjects: public SdoMsg
{
public:
  SDORxPDO2SetNumberOfMappedApplicationObjects(const uint32_t nodeId, const uint32_t number_of_mapped_objects):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1601, 0x00, number_of_mapped_objects)
  {}
  virtual ~SDORxPDO2SetNumberOfMappedApplicationObjects(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2ConfigureCOBID: public SdoMsg
{
public:
  SDORxPDO2ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1401, 0x01, 0x40000300 + nodeId)
  {}
  virtual ~SDORxPDO2ConfigureCOBID(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2Disable: public SdoMsg
{
public:
	SDORxPDO2Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1401, 0x01, 0xFFFFFFFF)
  {}
  virtual ~SDORxPDO2Disable(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2SetTransmissionType: public SdoMsg
{
public:
  SDORxPDO2SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1401, 0x02, type)
  {}
  virtual ~SDORxPDO2SetTransmissionType(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2SetMapping: public SdoMsg
{
public:
  SDORxPDO2SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1601, indexOfObject, object)
  {}
  virtual ~SDORxPDO2SetMapping(){}
};

/*********************************************************************
 * PDO 3 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3SetNumberOfMappedApplicationObjects: public SdoMsg
{
public:
  SDORxPDO3SetNumberOfMappedApplicationObjects(const uint32_t nodeId, const uint32_t number_of_mapped_objects):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1602, 0x00, number_of_mapped_objects)
  {}
  virtual ~SDORxPDO3SetNumberOfMappedApplicationObjects(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3ConfigureCOBID: public SdoMsg
{
public:
  SDORxPDO3ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1402, 0x01, 0x40000400 + nodeId)
  {}
  virtual ~SDORxPDO3ConfigureCOBID(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3Disable: public SdoMsg
{
public:
	SDORxPDO3Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1402, 0x01, 0xFFFFFFFF)
  {}
  virtual ~SDORxPDO3Disable(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3SetTransmissionType: public SdoMsg
{
public:
  SDORxPDO3SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1402, 0x02, type)
  {}
  virtual ~SDORxPDO3SetTransmissionType(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3SetMapping: public SdoMsg
{
public:
  SDORxPDO3SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1602, indexOfObject, object)
  {}
  virtual ~SDORxPDO3SetMapping(){}
};

/*********************************************************************
 * PDO 4 Parameter
 *********************************************************************/
//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4SetNumberOfMappedApplicationObjects: public SdoMsg
{
public:
  SDORxPDO4SetNumberOfMappedApplicationObjects(const uint32_t nodeId, const uint32_t number_of_mapped_objects):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1603, 0x00, number_of_mapped_objects)
  {}
  virtual ~SDORxPDO4SetNumberOfMappedApplicationObjects(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4ConfigureCOBID: public SdoMsg
{
public:
  SDORxPDO4ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1403, 0x01, 0x40000500 + nodeId)
  {}
  virtual ~SDORxPDO4ConfigureCOBID(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4Disable: public SdoMsg
{
public:
	SDORxPDO4Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1403, 0x01, 0xFFFFFFFF)
  {}
  virtual ~SDORxPDO4Disable(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4SetTransmissionType: public SdoMsg
{
public:
  SDORxPDO4SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1403, 0x02, type)
  {}
  virtual ~SDORxPDO4SetTransmissionType(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4SetMapping: public SdoMsg
{
public:
  SDORxPDO4SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1603, indexOfObject, object)
  {}
  virtual ~SDORxPDO4SetMapping(){}
};

//////////////////////////////////////////////////////////////////////////////
class SDOReadErrorRegister: public SdoMsg
{
public:
  SDOReadErrorRegister(const uint32_t nodeId):
	  SdoMsg(nodeId, SdoMsg::Command::READ, 0x1001, 0x00, 0x0)
  {}
  virtual ~SDOReadErrorRegister(){}

  std::string getErrorAsString() {
    uint8_t error = readuint8(4);
    std::string errors;

    if (error & 0x01 ) {
      errors += std::string{"generic error, "};
    }
    if (error & 0x02 ) {
      errors += std::string{"current, "};
    }
    if (error & 0x04 ) {
      errors += std::string{"voltage, "};
    }
    if (error & 0x08 ) {
      errors += std::string{"temperature, "};
    }
    if (error & 0x10 ) {
      errors += std::string{"communication error, "};
    }
    if (error & 0x20 ) {
      errors += std::string{"device profile specific, "};
    }
    // if (error & 0x40 )  // ignore

    if (error & 0x80 ) {
      errors += std::string{"manufacturer specific"};
    }
    return errors;
  }

};


} // namespace canopen
} /* namespace tcan */
