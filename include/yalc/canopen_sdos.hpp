//#pragma once

//#include <yalc/SDOWriteMsg.hpp>
//#include <yalc/SDOReadMsg.hpp>

//#define WRITE_1_BYTE 0x2f
//#define WRITE_2_BYTE 0x2b
//#define WRITE_4_BYTE 0x23

//namespace canopen {

////////////////////////////////////////////////////////////////////////////////
//class SDOWrite: public SDOWriteMsg
//{
//public:
//  SDOWrite(int nodeId, char length, int index, int subindex, int data):
//    SDOWriteMsg(nodeId, length, index, subindex, data)
//  {};
//  virtual ~SDOWrite(){};
//protected:

//  bool getErrorName(std::string& name) const {
//    const int32_t error = (inputMsg_->getValue()[4] + (inputMsg_->getValue()[5]<<8) + (inputMsg_->getValue()[6]<<16) + (inputMsg_->getValue()[7]<<24));
//    switch (error) {
//      case 0x00000000:
//        name = std::string{"No Communication Error"};
//        break;
//      case 0x05030000:
//        name = std::string{"Toggle Error"};
//        break;
//      case 0x05040000:
//        name = std::string{"SDO Time Out"};
//        break;
//      case 0x05040001:
//        name = std::string{"Client / Server Specifier Error"};
//        break;
//      case 0x05040005:
//        name = std::string{"Out of Memory Error"};
//        break;
//      case 0x06010000:
//        name = std::string{"Access Error"};
//        break;
//      case 0x06010001:
//        name = std::string{"Write Only"};
//        break;
//      case 0x06010002:
//        name = std::string{"Read Only"};
//        break;
//      case 0x06020000:
//        name = std::string{"Object does not exist Error"};
//        break;
//      case 0x06040041:
//        name = std::string{"PDO mapping Error"};
//        break;
//      case 0x06040042:
//        name = std::string{"PDO Length Error"};
//        break;
//      case 0x06040043:
//        name = std::string{"General Parameter Error"};
//        break;
//      case 0x06040047:
//        name = std::string{"General internal Incompatibility Error"};
//        break;
//      case 0x06060000:
//        name = std::string{"Hardware Error"};
//        break;
//      case 0x06070010:
//        name = std::string{"Service Parameter Error"};
//        break;
//      case 0x06070012:
//        name = std::string{"Service Parameter too long Error"};
//        break;
//      case 0x06070013:
//        name = std::string{"Service Parameter too short Error"};
//        break;
//      case 0x06090011:
//        name = std::string{"Object Subindex Error"};
//        break;
//      case 0x06090030:
//        name = std::string{"Value Range Error"};
//        break;
//      case 0x06090031:
//        name = std::string{"Value too high Error"};
//        break;
//      case 0x06090032:
//        name = std::string{"Value too low Error"};
//        break;
//      case 0x06090036:
//        name = std::string{"Maximum less Minimum Error"};
//        break;
//      case 0x08000000:
//        name = std::string{"General Error"};
//        break;
//      case 0x08000020:
//        name = std::string{"Transfer or store Error"};
//        break;
//      case 0x08000021:
//        name = std::string{"Local Control Error"};
//        break;
//      case 0x08000022:
//          name = std::string{"Wrong Device State"};
//          break;
//      default:
//        return false;
//    }
//    return true;
//  }


//  virtual void processReceivedMsg()
//  {
//    if (inputMsg_->getValue()[1] == outputMsg_->getValue()[1] && inputMsg_->getValue()[2] == outputMsg_->getValue()[2]) {
//      if (inputMsg_->getValue()[0] == 0x80)
//      {

//       std::string errorName;
//        if (getErrorName(errorName)) {
//          ROS_ERROR("SDO Write Error: Node: 0x%02X: %s! Error code: %02X%02X%02X%02X, "
//              "COB ID: %04X, Index: 0x%02X%02X, Subindex: 0x%02X, CAN message: %02X %02X %02X %02X %02X %02X %02X %02X",
//                     outputMsg_->getCOBId()-0x600,
//                     errorName.c_str(),
//                     inputMsg_->getValue()[7], inputMsg_->getValue()[6], inputMsg_->getValue()[5], inputMsg_->getValue()[4],
//                     outputMsg_->getCOBId(),
//                     inputMsg_->getValue()[2], inputMsg_->getValue()[1], // index
//                     inputMsg_->getValue()[3], // subindex
//                     inputMsg_->getValue()[0], inputMsg_->getValue()[1], inputMsg_->getValue()[2], inputMsg_->getValue()[3], inputMsg_->getValue()[4], inputMsg_->getValue()[5], inputMsg_->getValue()[6], inputMsg_->getValue()[7]);
//        }
//        else {
//          ///< Check for recData[0]==0x60! recData[0]==0x80 means an error happend
//          ROS_ERROR("SDO Error: Can't write! Error code: %02X%02X%02X%02X Output msg: COB ID: %04X Data: %02X %02X %02X %02X %02X %02X %02X %02X",
//                        inputMsg_->getValue()[7], inputMsg_->getValue()[6], inputMsg_->getValue()[5], inputMsg_->getValue()[4],
//                        outputMsg_->getCOBId(),
//                        inputMsg_->getValue()[0], inputMsg_->getValue()[1], inputMsg_->getValue()[2], inputMsg_->getValue()[3], inputMsg_->getValue()[4], inputMsg_->getValue()[5], inputMsg_->getValue()[6], inputMsg_->getValue()[7]);
//        }




//      }
//    }
//  }
//};


////////////////////////////////////////////////////////////////////////////////
//class SDORead: public SDOReadMsg
//{
//public:
//  SDORead(int inSDOSMId, int outSDOSMId, int nodeId, int index, int subindex):
//    SDOReadMsg(inSDOSMId, outSDOSMId, nodeId, index, subindex) //, isDataReceived_(false)
//  {};
//  virtual ~SDORead(){};

//  virtual void setIsReceived(bool flag) {
//    isReceived_ = flag;
//  }
//protected:

//  int data_;

//  virtual void processReceivedMsg()
//  {

////    if (inputMsg_->getValue()[0] == 0x80)
////    {
////      ///< Check for recData[0]==0x60! recData[0]==0x80 means an error happend
////      printf("\e[0;31mAN ERROR HAPPEND. CAN'T WRITE CHECK CHAP. 12.3 of the firmware specification for the error code: %02X%02X%02X%02X\n\e[0m",inputMsg_->getValue()[7], inputMsg_->getValue()[6], inputMsg_->getValue()[5], inputMsg_->getValue()[4]);
////    }

//    if (inputMsg_->getValue()[1] == outputMsg_->getValue()[1]
//          && inputMsg_->getValue()[2] == outputMsg_->getValue()[2]
//          && inputMsg_->getValue()[3] == outputMsg_->getValue()[3]) {
//      if (inputMsg_->getValue()[0] == 0x80)
//      {
//        ///< Check for recData[0]==0x60! recData[0]==0x80 means an error happend
//        printf("\e[0;31mAN ERROR HAPPEND. CAN'T WRITE CHECK CHAP. 12.3 of the firmware specification for the error code: %02X%02X%02X%02X\n\e[0m",inputMsg_->getValue()[7], inputMsg_->getValue()[6], inputMsg_->getValue()[5], inputMsg_->getValue()[4]);
//      }
//    }

//    data_ = (inputMsg_->getValue()[4] + (inputMsg_->getValue()[5]<<8) + (inputMsg_->getValue()[6]<<16) + (inputMsg_->getValue()[7]<<24));

//  }

//};



///** *********************************************************************
//----------------------------- NMT Server -------------------------------
//********************************************************************* **/
////////////////////////////////////////////////////////////////////////////////
//class SDONMTEnterPreOperational: public SDOWrite
//{
//public:
//  SDONMTEnterPreOperational(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, 0, 0, 0, 0)
//  {
//    outputMsg_->setCOBId(0x00);
//    inputMsg_->setCOBId(0x00);
//    int Length[8] = {1, 1, 0, 0, 0, 0, 0, 0};
//    int Value[8] = {0x80,
//            nodeId_,
//            0x00,
//            0x00,
//            0x00,
//            0x00,
//            0x00,
//            0x00};
//    outputMsg_->setLength(Length);
//    outputMsg_->setValue(Value);
//    outputMsg_->setFlag(1);
//    isReceived_ = true;

//  };
//  virtual ~SDONMTEnterPreOperational(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDONMTStartRemoteNode: public SDOWrite
//{
//public:
//  SDONMTStartRemoteNode(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, 0, 0, 0, 0)
//  {
//    outputMsg_->setCOBId(0x00);
//    inputMsg_->setCOBId(0x00);
//    int Length[8] = {1, 1, 0, 0, 0, 0, 0, 0};
//    int Value[8] = {0x01,
//            nodeId_,
//            0x00,
//            0x00,
//            0x00,
//            0x00,
//            0x00,
//            0x00};
//    outputMsg_->setLength(Length);
//    outputMsg_->setValue(Value);
//    outputMsg_->setFlag(1);
//    isReceived_ = true;
//  };
//  virtual ~SDONMTStartRemoteNode(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDONMTStopRemoteNode: public SDOWrite
//{
//public:
//  SDONMTStopRemoteNode(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, 0, 0, 0, 0)
//  {

//    outputMsg_->setCOBId(0x00);
//    inputMsg_->setCOBId(0x00);
//    int Length[8] = {1, 1, 0, 0, 0, 0, 0, 0};
//    int Value[8] = {0x02,
//            nodeId,
//            0x00,
//            0x00,
//            0x00,
//            0x00,
//            0x00,
//            0x00};
//    outputMsg_->setLength(Length);
//    outputMsg_->setValue(Value);
//    outputMsg_->setFlag(1);
//    isReceived_ = true;

//  };
//  virtual ~SDONMTStopRemoteNode(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDONMTResetCommunication: public SDOWrite
//{
//public:
//  SDONMTResetCommunication(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, 0, 0, 0, 0)
//  {
//    outputMsg_->setCOBId(0x00);
//    inputMsg_->setCOBId(0x00);
//    int Length[8] = {1, 1, 0, 0, 0, 0, 0, 0};
//    int Value[8] = {0x82,
//            nodeId_,
//            0x00,
//            0x00,
//            0x00,
//            0x00,
//            0x00,
//            0x00};
//    outputMsg_->setLength(Length);
//    outputMsg_->setValue(Value);
//    outputMsg_->setFlag(1);
//    isReceived_ = true;

//  };
//  virtual ~SDONMTResetCommunication(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDONMTResetNode: public SDOWrite
//{
//public:
//  SDONMTResetNode(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, 0, 0, 0, 0)
//  {
//    outputMsg_->setCOBId(0x00);
//    inputMsg_->setCOBId(0x00);
//    int Length[8] = {1, 1, 0, 0, 0, 0, 0, 0};
//    int Value[8] = {0x81,
//            nodeId_,
//            0x00,
//            0x00,
//            0x00,
//            0x00,
//            0x00,
//            0x00};
//    outputMsg_->setLength(Length);
//    outputMsg_->setValue(Value);
//    outputMsg_->setFlag(1);
//    isReceived_ = true;

//  };
//  virtual ~SDONMTResetNode(){};
//};



///** *********************************************************************
//----------------------------- Communication -----------------------------
//********************************************************************* **/
////////////////////////////////////////////////////////////////////////////////
//class SDOSetRS232Baudrate: public SDOWrite
//{
//public:
//  SDOSetRS232Baudrate(int inSDOSMId, int outSDOSMId, int nodeId, int baudrate):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x2002, 0x00, baudrate)
//  {};
//  virtual ~SDOSetRS232Baudrate(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOSetCANBitrate: public SDOWrite
//{
//public:
//  SDOSetCANBitrate(int inSDOSMId, int outSDOSMId, int nodeId, int bitrate):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x2001, 0x00, bitrate)
//  {};
//  virtual ~SDOSetCANBitrate(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOSetAbortConnectionOptionCode: public SDOWrite
//{
//public:
//  SDOSetAbortConnectionOptionCode(int inSDOSMId, int outSDOSMId, int nodeId, int value):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6007, 0x00, value)
//  {};
//  virtual ~SDOSetAbortConnectionOptionCode(){};
//};


///** *********************************************************************
//----------------------------- Initialization ----------------------------
//********************************************************************* **/
////////////////////////////////////////////////////////////////////////////////
//class SDOControlword: public SDOWrite
//{
//public:
//  SDOControlword(int inSDOSMId, int outSDOSMId, int nodeId, int controlword):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6040, 0x00, controlword)
//  {};
//  virtual ~SDOControlword(){};
//};
////////////////////////////////////////////////////////////////////////////////
//class SDOShutdown: public SDOWrite
//{
//public:
//  SDOShutdown(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6040, 0x00, 0x06)
//  {};
//  virtual ~SDOShutdown(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOSwitchOn: public SDOWrite
//{
//public:
//  SDOSwitchOn(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6040, 0x00, 0x07)
//  {};
//  virtual ~SDOSwitchOn(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOEnableOperation: public SDOWrite
//{
//public:
//  SDOEnableOperation(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6040, 0x00, 0x0F)
//  {};
//  virtual ~SDOEnableOperation(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDODisableOperation: public SDOWrite
//{
//public:
//  SDODisableOperation(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6040, 0x00, 0x07)
//  {};
//  virtual ~SDODisableOperation(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOFaultReset: public SDOWrite
//{
//public:
//  SDOFaultReset(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x6040, 0x00, 0x80)
//  {};
//  virtual ~SDOFaultReset(){};
//};

///************************************************************************
//------------------------ Digital Inputs ---------------------------------
//********************************************************************* **/
////////////////////////////////////////////////////////////////////////////////
//class SDOSetDigitalInputFunctionalitiesMask: public SDOWrite
//{
//public:
//  SDOSetDigitalInputFunctionalitiesMask(int inSDOSMId, int outSDOSMId, int nodeId, int value):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x2071, 0x02, value)
//  {};
//  virtual ~SDOSetDigitalInputFunctionalitiesMask(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOSetDigitalInputFunctionalitiesPolarity
//: public SDOWrite
//{
//public:
//  SDOSetDigitalInputFunctionalitiesPolarity(int inSDOSMId, int outSDOSMId, int nodeId, int value):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x2071, 0x03, value)
//  {};
//  virtual ~SDOSetDigitalInputFunctionalitiesPolarity(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOSetDigitalInputFunctionalitiesExecutionMask
//: public SDOWrite
//{
//public:
//  SDOSetDigitalInputFunctionalitiesExecutionMask(int inSDOSMId, int outSDOSMId, int nodeId, int value):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x2071, 0x04, value)
//  {};
//  virtual ~SDOSetDigitalInputFunctionalitiesExecutionMask(){};
//};


///***********************************************************************
//-------------------- Life guard and heartbeat control-------------------
//********************************************************************* **/
////////////////////////////////////////////////////////////////////////////////
//class SDOSetGuardTime: public SDOWrite
//{
//public:
//  SDOSetGuardTime(int inSDOSMId, int outSDOSMId, int nodeId, int time_ms):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x100C, 0x00, time_ms)
//  {};
//  virtual ~SDOSetGuardTime(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOSetLifeTimeFactor: public SDOWrite
//{
//public:
//  SDOSetLifeTimeFactor(int inSDOSMId, int outSDOSMId, int nodeId, int factor):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x100D, 0x00, factor)
//  {};
//  virtual ~SDOSetLifeTimeFactor(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOWriteProducerHeartbeatTime: public SDOWrite
//{
//public:
//  SDOWriteProducerHeartbeatTime(int inSDOSMId, int outSDOSMId, int nodeId, int time_ms):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x1017, 0x00, time_ms)
//  {};
//  virtual ~SDOWriteProducerHeartbeatTime(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOReadProducerHeartbeatTime: public SDORead
//{
//public:
//  SDOReadProducerHeartbeatTime(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDORead(inSDOSMId, outSDOSMId, nodeId, 0x1017, 0x00),
//    time_(0)
//  {};


//  virtual void processReceivedMsg()
//  {
//    SDORead::processReceivedMsg();
//    time_ = readuint16();
//  }

//  virtual ~SDOReadProducerHeartbeatTime(){};

//  uint16_t getTime() const {
//    return time_;
//  }

//private:
//  uint16_t time_;
//};

///***********************************************************************
//------------------------------ Utilities --------------------------------
//********************************************************************* **/
////////////////////////////////////////////////////////////////////////////////
//class SDOSetCOBIDSYNC: public SDOWrite
//{
//public:
//  SDOSetCOBIDSYNC(int inSDOSMId, int outSDOSMId, int nodeId, int ID):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1005, 0x00, ID)
//  {};
//  virtual ~SDOSetCOBIDSYNC(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOSaveAllParameters: public SDOWrite
//{
//public:
//  SDOSaveAllParameters(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1010, 0x01, 0x65766173)
//  {};
//  virtual ~SDOSaveAllParameters(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORestoreAllDefaultParameters: public SDOWrite
//{
//public:
//  SDORestoreAllDefaultParameters(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1011, 0x01, 0x64616F6C)
//  {};
//  virtual ~SDORestoreAllDefaultParameters(){};
//};


///** *********************************************************************
//------------------------------ Tx PDO's ---------------------------------
//********************************************************************* **/

///*********************************************************************
// * PDO 1 Parameter
// *********************************************************************/

////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO1Disable: public SDOWrite
//{
//public:
//  SDOTxPDO1Disable(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1800, 0x01, 0x80000180 + nodeId)
//  {};
//  virtual ~SDOTxPDO1Disable(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO1SetInhibitTime: public SDOWrite
//{
//public:
//  SDOTxPDO1SetInhibitTime(int inSDOSMId, int outSDOSMId, int nodeId, int time_100us):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_2_BYTE, 0x1800, 0x03, time_100us)
//  {};
//  virtual ~SDOTxPDO1SetInhibitTime(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO1SetNumberOfMappedApplicationObjects: public SDOWrite
//{
//public:
//  SDOTxPDO1SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1A00, 0x00, number_of_mapped_objects)
//  {};
//  virtual ~SDOTxPDO1SetNumberOfMappedApplicationObjects(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO1ConfigureCOBID: public SDOWrite
//{
//public:
//  SDOTxPDO1ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1800, 0x01, 0x40000180 + nodeId)
//  {};
//  virtual ~SDOTxPDO1ConfigureCOBID(){};
//};
////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO1SetTransmissionType: public SDOWrite
//{
//public:
//  SDOTxPDO1SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1800, 0x02, type)
//  {};
//  virtual ~SDOTxPDO1SetTransmissionType(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO1SetMapping: public SDOWrite
//{
//public:
//  SDOTxPDO1SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1A00, indexOfObject, object)
//  {};
//  virtual ~SDOTxPDO1SetMapping(){};
//};


///*********************************************************************
// * PDO 2 Parameter
// *********************************************************************/
////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO2SetNumberOfMappedApplicationObjects: public SDOWrite
//{
//public:
//  SDOTxPDO2SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1A01, 0x00, number_of_mapped_objects)
//  {};
//  virtual ~SDOTxPDO2SetNumberOfMappedApplicationObjects(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO2ConfigureCOBID: public SDOWrite
//{
//public:
//  SDOTxPDO2ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1801, 0x01, 0x40000280 + nodeId)
//  {};
//  virtual ~SDOTxPDO2ConfigureCOBID(){};
//};
////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO2Disable: public SDOWrite
//{
//public:
//  SDOTxPDO2Disable(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1801, 0x01, 0x80000280 + nodeId)
//  {};
//  virtual ~SDOTxPDO2Disable(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO2SetTransmissionType: public SDOWrite
//{
//public:
//  SDOTxPDO2SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1801, 0x02, type)
//  {};
//  virtual ~SDOTxPDO2SetTransmissionType(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO2SetMapping: public SDOWrite
//{
//public:
//  SDOTxPDO2SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1A01, indexOfObject, object)
//  {};
//  virtual ~SDOTxPDO2SetMapping(){};
//};

///*********************************************************************
// * PDO 3 Parameter
// *********************************************************************/
////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO3SetNumberOfMappedApplicationObjects: public SDOWrite
//{
//public:
//  SDOTxPDO3SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1A02, 0x00, number_of_mapped_objects)
//  {};
//  virtual ~SDOTxPDO3SetNumberOfMappedApplicationObjects(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO3ConfigureCOBID: public SDOWrite
//{
//public:
//  SDOTxPDO3ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1802, 0x01, 0x40000380 + nodeId)
//  {};
//  virtual ~SDOTxPDO3ConfigureCOBID(){};
//};
////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO3Disable: public SDOWrite
//{
//public:
//  SDOTxPDO3Disable(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1802, 0x01, 0x80000380 + nodeId)
//  {};
//  virtual ~SDOTxPDO3Disable(){};
//};
////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO3SetTransmissionType: public SDOWrite
//{
//public:
//  SDOTxPDO3SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1802, 0x02, type)
//  {};
//  virtual ~SDOTxPDO3SetTransmissionType(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO3SetMapping: public SDOWrite
//{
//public:
//  SDOTxPDO3SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1A02, indexOfObject, object)
//  {};
//  virtual ~SDOTxPDO3SetMapping(){};
//};

///*********************************************************************
// * PDO 4 Parameter
// *********************************************************************/
////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO4SetNumberOfMappedApplicationObjects: public SDOWrite
//{
//public:
//  SDOTxPDO4SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1A03, 0x00, number_of_mapped_objects)
//  {};
//  virtual ~SDOTxPDO4SetNumberOfMappedApplicationObjects(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO4ConfigureCOBID: public SDOWrite
//{
//public:
//  SDOTxPDO4ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1803, 0x01, 0x40000480 + nodeId)
//  {};
//  virtual ~SDOTxPDO4ConfigureCOBID(){};
//};
////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO4Disable: public SDOWrite
//{
//public:
//	SDOTxPDO4Disable(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1803, 0x01, 0xFFFFFFFF)
//  {};
//  virtual ~SDOTxPDO4Disable(){};
//};
////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO4SetTransmissionType: public SDOWrite
//{
//public:
//  SDOTxPDO4SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1803, 0x02, type)
//  {};
//  virtual ~SDOTxPDO4SetTransmissionType(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOTxPDO4SetMapping: public SDOWrite
//{
//public:
//  SDOTxPDO4SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1A03, indexOfObject, object)
//  {};
//  virtual ~SDOTxPDO4SetMapping(){};
//};


///** *********************************************************************
//------------------------------ Rx PDO's ---------------------------------
//********************************************************************* **/
///*********************************************************************
// * PDO 1 Parameter
// *********************************************************************/
////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO1SetNumberOfMappedApplicationObjects: public SDOWrite
//{
//public:
//  SDORxPDO1SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1600, 0x00, number_of_mapped_objects)
//  {};
//  virtual ~SDORxPDO1SetNumberOfMappedApplicationObjects(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO1ConfigureCOBID: public SDOWrite
//{
//public:
//  SDORxPDO1ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1400, 0x01, 0x40000200 + nodeId)
//  {};
//  virtual ~SDORxPDO1ConfigureCOBID(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO1Disable: public SDOWrite
//{
//public:
//  SDORxPDO1Disable(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1400, 0x01, 0xFFFFFFFF)
//  {};
//  virtual ~SDORxPDO1Disable(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO1SetTransmissionType: public SDOWrite
//{
//public:
//  SDORxPDO1SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1400, 0x02, type)
//  {};
//  virtual ~SDORxPDO1SetTransmissionType(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO1SetMapping: public SDOWrite
//{
//public:
//  SDORxPDO1SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1600, indexOfObject, object)
//  {};
//  virtual ~SDORxPDO1SetMapping(){};
//};

///*********************************************************************
// * PDO 2 Parameter
// *********************************************************************/
////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO2SetNumberOfMappedApplicationObjects: public SDOWrite
//{
//public:
//  SDORxPDO2SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1601, 0x00, number_of_mapped_objects)
//  {};
//  virtual ~SDORxPDO2SetNumberOfMappedApplicationObjects(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO2ConfigureCOBID: public SDOWrite
//{
//public:
//  SDORxPDO2ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1401, 0x01, 0x40000300 + nodeId)
//  {};
//  virtual ~SDORxPDO2ConfigureCOBID(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO2Disable: public SDOWrite
//{
//public:
//	SDORxPDO2Disable(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1401, 0x01, 0xFFFFFFFF)
//  {};
//  virtual ~SDORxPDO2Disable(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO2SetTransmissionType: public SDOWrite
//{
//public:
//  SDORxPDO2SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1401, 0x02, type)
//  {};
//  virtual ~SDORxPDO2SetTransmissionType(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO2SetMapping: public SDOWrite
//{
//public:
//  SDORxPDO2SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1601, indexOfObject, object)
//  {};
//  virtual ~SDORxPDO2SetMapping(){};
//};

///*********************************************************************
// * PDO 3 Parameter
// *********************************************************************/
////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO3SetNumberOfMappedApplicationObjects: public SDOWrite
//{
//public:
//  SDORxPDO3SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1602, 0x00, number_of_mapped_objects)
//  {};
//  virtual ~SDORxPDO3SetNumberOfMappedApplicationObjects(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO3ConfigureCOBID: public SDOWrite
//{
//public:
//  SDORxPDO3ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1402, 0x01, 0x40000400 + nodeId)
//  {};
//  virtual ~SDORxPDO3ConfigureCOBID(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO3Disable: public SDOWrite
//{
//public:
//	SDORxPDO3Disable(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1402, 0x01, 0xFFFFFFFF)
//  {};
//  virtual ~SDORxPDO3Disable(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO3SetTransmissionType: public SDOWrite
//{
//public:
//  SDORxPDO3SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1402, 0x02, type)
//  {};
//  virtual ~SDORxPDO3SetTransmissionType(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO3SetMapping: public SDOWrite
//{
//public:
//  SDORxPDO3SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1602, indexOfObject, object)
//  {};
//  virtual ~SDORxPDO3SetMapping(){};
//};

///*********************************************************************
// * PDO 4 Parameter
// *********************************************************************/
////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO4SetNumberOfMappedApplicationObjects: public SDOWrite
//{
//public:
//  SDORxPDO4SetNumberOfMappedApplicationObjects(int inSDOSMId, int outSDOSMId, int nodeId, int number_of_mapped_objects):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1603, 0x00, number_of_mapped_objects)
//  {};
//  virtual ~SDORxPDO4SetNumberOfMappedApplicationObjects(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO4ConfigureCOBID: public SDOWrite
//{
//public:
//  SDORxPDO4ConfigureCOBID(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1403, 0x01, 0x40000500 + nodeId)
//  {};
//  virtual ~SDORxPDO4ConfigureCOBID(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO4Disable: public SDOWrite
//{
//public:
//	SDORxPDO4Disable(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1403, 0x01, 0xFFFFFFFF)
//  {};
//  virtual ~SDORxPDO4Disable(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO4SetTransmissionType: public SDOWrite
//{
//public:
//  SDORxPDO4SetTransmissionType(int inSDOSMId, int outSDOSMId, int nodeId, int type):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_1_BYTE, 0x1403, 0x02, type)
//  {};
//  virtual ~SDORxPDO4SetTransmissionType(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDORxPDO4SetMapping: public SDOWrite
//{
//public:
//  SDORxPDO4SetMapping(int inSDOSMId, int outSDOSMId, int nodeId, int indexOfObject, int object):
//    SDOWrite(inSDOSMId, outSDOSMId, nodeId, WRITE_4_BYTE, 0x1603, indexOfObject, object)
//  {};
//  virtual ~SDORxPDO4SetMapping(){};
//};

////////////////////////////////////////////////////////////////////////////////
//class SDOReadErrorRegister: public SDORead
//{
//public:
//  SDOReadErrorRegister(int inSDOSMId, int outSDOSMId, int nodeId):
//    SDORead(inSDOSMId, outSDOSMId, nodeId, 0x1001, 0x00)
//  {};
//  virtual ~SDOReadErrorRegister(){};

//  virtual void processReceivedMsg()
//  {
//    SDORead::processReceivedMsg();
//  }

//  std::string getErrorAsString() {
//    uint8_t error = readuint8();
//    std::string errors;

//    if (error & 0x01 ) {
//      errors += std::string{"generic error, "};
//    }
//    if (error & 0x02 ) {
//      errors += std::string{"current, "};
//    }
//    if (error & 0x04 ) {
//      errors += std::string{"voltage, "};
//    }
//    if (error & 0x08 ) {
//      errors += std::string{"temperature, "};
//    }
//    if (error & 0x10 ) {
//      errors += std::string{"communication error, "};
//    }
//    if (error & 0x20 ) {
//      errors += std::string{"device profile specific, "};
//    }
//    // if (error & 0x40 )  // ignore

//    if (error & 0x80 ) {
//      errors += std::string{"manufacturer specific"};
//    }
//    return errors;
//  }

//};


//} // namespace
