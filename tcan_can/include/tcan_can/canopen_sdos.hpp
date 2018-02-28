#pragma once

#include "tcan_can/SdoMsg.hpp"

namespace tcan_can {
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
  ~SDOSetRS232Baudrate() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetCANBitrate: public SdoMsg
{
public:
  SDOSetCANBitrate(const uint32_t nodeId, const uint32_t bitrate):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x2001, 0x00, bitrate)
  {}
  ~SDOSetCANBitrate() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetAbortConnectionOptionCode: public SdoMsg
{
public:
  SDOSetAbortConnectionOptionCode(const uint32_t nodeId, const uint32_t value):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6007, 0x00, value)
  {}
  ~SDOSetAbortConnectionOptionCode() override = default;
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
  ~SDOControlword() override = default;
};
//////////////////////////////////////////////////////////////////////////////
class SDOShutdown: public SdoMsg
{
public:
  SDOShutdown(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6040, 0x00, 0x06)
  {}
  ~SDOShutdown() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOSwitchOn: public SdoMsg
{
public:
  SDOSwitchOn(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6040, 0x00, 0x07)
  {}
  ~SDOSwitchOn() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOEnableOperation: public SdoMsg
{
public:
  SDOEnableOperation(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6040, 0x00, 0x0F)
  {}
  ~SDOEnableOperation() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDODisableOperation: public SdoMsg
{
public:
  SDODisableOperation(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6040, 0x00, 0x07)
  {}
  ~SDODisableOperation() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOFaultReset: public SdoMsg
{
public:
  SDOFaultReset(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x6040, 0x00, 0x80)
  {}
  ~SDOFaultReset() override = default;
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
  ~SDOSetDigitalInputFunctionalitiesMask() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetDigitalInputFunctionalitiesPolarity
: public SdoMsg
{
public:
  SDOSetDigitalInputFunctionalitiesPolarity(const uint32_t nodeId, const uint32_t value):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x2071, 0x03, value)
  {}
  ~SDOSetDigitalInputFunctionalitiesPolarity() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetDigitalInputFunctionalitiesExecutionMask
: public SdoMsg
{
public:
  SDOSetDigitalInputFunctionalitiesExecutionMask(const uint32_t nodeId, const uint32_t value):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x2071, 0x04, value)
  {}
  ~SDOSetDigitalInputFunctionalitiesExecutionMask() override = default;
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
  ~SDOSetGuardTime() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOSetLifeTimeFactor: public SdoMsg
{
public:
  SDOSetLifeTimeFactor(const uint32_t nodeId, const uint32_t factor):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x100D, 0x00, factor)
  {}
  ~SDOSetLifeTimeFactor() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOWriteProducerHeartbeatTime: public SdoMsg
{
public:
  SDOWriteProducerHeartbeatTime(const uint32_t nodeId, const uint32_t time_ms):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x1017, 0x00, time_ms)
  {}
  ~SDOWriteProducerHeartbeatTime() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOReadProducerHeartbeatTime: public SdoMsg
{
public:
  SDOReadProducerHeartbeatTime(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::READ, 0x1017, 0x00, 0x0)
  {}

  ~SDOReadProducerHeartbeatTime() override = default;
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
  ~SDOSetCOBIDSYNC() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOSaveAllParameters: public SdoMsg
{
public:
  SDOSaveAllParameters(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1010, 0x01, 0x65766173)
  {}
  ~SDOSaveAllParameters() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORestoreAllDefaultParameters: public SdoMsg
{
public:
  SDORestoreAllDefaultParameters(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1011, 0x01, 0x64616F6C)
  {}
  ~SDORestoreAllDefaultParameters() override = default;
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
  ~SDOTxPDO1Disable() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1SetInhibitTime: public SdoMsg
{
public:
  SDOTxPDO1SetInhibitTime(const uint32_t nodeId, const uint32_t time_100us):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x1800, 0x03, time_100us)
  {}
  ~SDOTxPDO1SetInhibitTime() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1SetNumberOfMappedApplicationObjects: public SdoMsg
{
public:
  SDOTxPDO1SetNumberOfMappedApplicationObjects(const uint32_t nodeId, const uint32_t number_of_mapped_objects):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1A00, 0x00, number_of_mapped_objects)
  {}
  ~SDOTxPDO1SetNumberOfMappedApplicationObjects() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1ConfigureCOBID: public SdoMsg
{
public:
  SDOTxPDO1ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1800, 0x01, 0x40000180 + nodeId)
  {}
  ~SDOTxPDO1ConfigureCOBID() override = default;
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1SetTransmissionType: public SdoMsg
{
public:
  SDOTxPDO1SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1800, 0x02, type)
  {}
  ~SDOTxPDO1SetTransmissionType() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO1SetMapping: public SdoMsg
{
public:
  SDOTxPDO1SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1A00, indexOfObject, object)
  {}
  ~SDOTxPDO1SetMapping() override = default;
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
  ~SDOTxPDO2SetNumberOfMappedApplicationObjects() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2ConfigureCOBID: public SdoMsg
{
public:
  SDOTxPDO2ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1801, 0x01, 0x40000280 + nodeId)
  {}
  ~SDOTxPDO2ConfigureCOBID() override = default;
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2Disable: public SdoMsg
{
public:
  SDOTxPDO2Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1801, 0x01, 0x80000280 + nodeId)
  {}
  ~SDOTxPDO2Disable() override = default;
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2SetInhibitTime: public SdoMsg
{
public:
    SDOTxPDO2SetInhibitTime(const uint32_t nodeId, const uint32_t time_100us):
            SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x1801, 0x03, time_100us)
    {}
    ~SDOTxPDO2SetInhibitTime() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2SetTransmissionType: public SdoMsg
{
public:
  SDOTxPDO2SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1801, 0x02, type)
  {}
  ~SDOTxPDO2SetTransmissionType() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO2SetMapping: public SdoMsg
{
public:
  SDOTxPDO2SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1A01, indexOfObject, object)
  {}
  ~SDOTxPDO2SetMapping() override = default;
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
  ~SDOTxPDO3SetNumberOfMappedApplicationObjects() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3ConfigureCOBID: public SdoMsg
{
public:
  SDOTxPDO3ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1802, 0x01, 0x40000380 + nodeId)
  {}
  ~SDOTxPDO3ConfigureCOBID() override = default;
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3Disable: public SdoMsg
{
public:
  SDOTxPDO3Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1802, 0x01, 0x80000380 + nodeId)
  {}
  ~SDOTxPDO3Disable() override = default;
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3SetInhibitTime: public SdoMsg
{
public:
    SDOTxPDO3SetInhibitTime(const uint32_t nodeId, const uint32_t time_100us):
            SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x1802, 0x03, time_100us)
    {}
    ~SDOTxPDO3SetInhibitTime() override = default;
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3SetTransmissionType: public SdoMsg
{
public:
  SDOTxPDO3SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1802, 0x02, type)
  {}
  ~SDOTxPDO3SetTransmissionType() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO3SetMapping: public SdoMsg
{
public:
  SDOTxPDO3SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1A02, indexOfObject, object)
  {}
  ~SDOTxPDO3SetMapping() override = default;
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
  ~SDOTxPDO4SetNumberOfMappedApplicationObjects() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4ConfigureCOBID: public SdoMsg
{
public:
  SDOTxPDO4ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1803, 0x01, 0x40000480 + nodeId)
  {}
  ~SDOTxPDO4ConfigureCOBID() override = default;
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4Disable: public SdoMsg
{
public:
	SDOTxPDO4Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1803, 0x01, 0xFFFFFFFF)
  {}
  ~SDOTxPDO4Disable() override = default;
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4SetInhibitTime: public SdoMsg
{
public:
    SDOTxPDO4SetInhibitTime(const uint32_t nodeId, const uint32_t time_100us):
            SdoMsg(nodeId, SdoMsg::Command::WRITE_2_BYTE, 0x1803, 0x03, time_100us)
    {}
    ~SDOTxPDO4SetInhibitTime() override = default;
};
//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4SetTransmissionType: public SdoMsg
{
public:
  SDOTxPDO4SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1803, 0x02, type)
  {}
  ~SDOTxPDO4SetTransmissionType() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOTxPDO4SetMapping: public SdoMsg
{
public:
  SDOTxPDO4SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1A03, indexOfObject, object)
  {}
  ~SDOTxPDO4SetMapping() override = default;
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
  ~SDORxPDO1SetNumberOfMappedApplicationObjects() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1ConfigureCOBID: public SdoMsg
{
public:
  SDORxPDO1ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1400, 0x01, 0x40000200 + nodeId)
  {}
  ~SDORxPDO1ConfigureCOBID() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1Disable: public SdoMsg
{
public:
  SDORxPDO1Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1400, 0x01, 0xFFFFFFFF)
  {}
  ~SDORxPDO1Disable() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1SetTransmissionType: public SdoMsg
{
public:
  SDORxPDO1SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1400, 0x02, type)
  {}
  ~SDORxPDO1SetTransmissionType() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO1SetMapping: public SdoMsg
{
public:
  SDORxPDO1SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1600, indexOfObject, object)
  {}
  ~SDORxPDO1SetMapping() override = default;
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
  ~SDORxPDO2SetNumberOfMappedApplicationObjects() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2ConfigureCOBID: public SdoMsg
{
public:
  SDORxPDO2ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1401, 0x01, 0x40000300 + nodeId)
  {}
  ~SDORxPDO2ConfigureCOBID() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2Disable: public SdoMsg
{
public:
	SDORxPDO2Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1401, 0x01, 0xFFFFFFFF)
  {}
  ~SDORxPDO2Disable() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2SetTransmissionType: public SdoMsg
{
public:
  SDORxPDO2SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1401, 0x02, type)
  {}
  ~SDORxPDO2SetTransmissionType() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO2SetMapping: public SdoMsg
{
public:
  SDORxPDO2SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1601, indexOfObject, object)
  {}
  ~SDORxPDO2SetMapping() override = default;
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
  ~SDORxPDO3SetNumberOfMappedApplicationObjects() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3ConfigureCOBID: public SdoMsg
{
public:
  SDORxPDO3ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1402, 0x01, 0x40000400 + nodeId)
  {}
  ~SDORxPDO3ConfigureCOBID() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3Disable: public SdoMsg
{
public:
	SDORxPDO3Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1402, 0x01, 0xFFFFFFFF)
  {}
  ~SDORxPDO3Disable() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3SetTransmissionType: public SdoMsg
{
public:
  SDORxPDO3SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1402, 0x02, type)
  {}
  ~SDORxPDO3SetTransmissionType() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO3SetMapping: public SdoMsg
{
public:
  SDORxPDO3SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1602, indexOfObject, object)
  {}
  ~SDORxPDO3SetMapping() override = default;
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
  ~SDORxPDO4SetNumberOfMappedApplicationObjects() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4ConfigureCOBID: public SdoMsg
{
public:
  SDORxPDO4ConfigureCOBID(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1403, 0x01, 0x40000500 + nodeId)
  {}
  ~SDORxPDO4ConfigureCOBID() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4Disable: public SdoMsg
{
public:
	SDORxPDO4Disable(const uint32_t nodeId):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1403, 0x01, 0xFFFFFFFF)
  {}
  ~SDORxPDO4Disable() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4SetTransmissionType: public SdoMsg
{
public:
  SDORxPDO4SetTransmissionType(const uint32_t nodeId, const uint32_t type):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_1_BYTE, 0x1403, 0x02, type)
  {}
  ~SDORxPDO4SetTransmissionType() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDORxPDO4SetMapping: public SdoMsg
{
public:
  SDORxPDO4SetMapping(const uint32_t nodeId, const uint8_t indexOfObject, const uint32_t object):
    SdoMsg(nodeId, SdoMsg::Command::WRITE_4_BYTE, 0x1603, indexOfObject, object)
  {}
  ~SDORxPDO4SetMapping() override = default;
};

//////////////////////////////////////////////////////////////////////////////
class SDOReadErrorRegister: public SdoMsg
{
public:
  SDOReadErrorRegister(const uint32_t nodeId):
	  SdoMsg(nodeId, SdoMsg::Command::READ, 0x1001, 0x00, 0x0)
  {}
  ~SDOReadErrorRegister() override = default;

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
} /* namespace tcan_can */
