#include <signal.h>
#include <unordered_map>

#include "tcan/EtherCatBus.hpp"
#include "tcan/EtherCatBusManager.hpp"

#define USE_SYNCHRONOUS_MODE

/*
namespace tcan_example {
class TcpManager : public tcan::IpBusManager {
public:
	enum class IpId : unsigned int {
		DEVICE1=0
	};

	typedef std::unordered_map<unsigned int, TcpConnection*> ConnectionContainer;

	TcpManager():
	    tcan::IpBusManager(),
	    connectionContainer_()
	{
		addConnection(IpId::DEVICE1, "192.168.0.100", 9999);
	}

	virtual ~TcpManager()
	{
		// close Buses (especially their threads!) here, so that the receiveThread does not try to call a callback of a already destructed object (parseIncomingSync(..) in this case)
		closeBuses();
	}

	void addConnection(const IpId ipId, const std::string& host, const unsigned int port) {
		tcan::IpBusOptions* options = new tcan::IpBusOptions(host, port);
#ifdef USE_SYNCHRONOUS_MODE
		options->asynchronous = false;
#endif

		auto connection = new TcpConnection(options);
		if(!addBus( connection )) {
			std::cout << "failed to add Bus " << host << std::endl;
			exit(-1);
		}

		connectionContainer_.insert({static_cast<unsigned int>(ipId), connection});
	}

	ConnectionContainer getConnectionContainer() { return connectionContainer_; }

protected:
	ConnectionContainer connectionContainer_;
};

} // namespace tcan_example
*/





typedef enum ECAT_DSP402_COMMAND_TYPE
{
    SWITCH_ON           = 0x00,
    SHUTDOWN            ,
    DISABLE_VOLTAGE     ,
    ENABLE_VOLTAGE      ,
    QUICK_STOP          ,
    DISABLE_OPERATION   ,
    ENABLE_OPERATION    ,
    FAULT_RESET         ,
    HALT                ,
    HALT_RESET          ,
    CLEAR_CONTROLWORD   ,

} dsp402_command_e;

// Note: Bit field ordering depends on the Endianness which is implementation dependent.
typedef struct DSP402_STATUSWORD_BITS
{
    uint16_t ready_to_switch_on:1;
    uint16_t switched_on:1;
    uint16_t operation_enabled:1;
    uint16_t fault:1;
    uint16_t voltage_enabled:1;
    uint16_t quick_stop:1;
    uint16_t switch_on_disabled:1;
    uint16_t warning:1;
    uint16_t manufacturer_specific_0:1;
    uint16_t remote:1;
    uint16_t operation_mode_specific_0:1;
    uint16_t internal_limit_active:1;
    uint16_t operation_mode_specific_1:1;
    uint16_t operation_mode_specific_2:1;
    uint16_t manufacturer_specific_1:2;

} dsp402_statusword_bits_t;

typedef union DSP402_STATUSWORD_TYPE
{
    uint16_t all;
    dsp402_statusword_bits_t bits;

} dsp402_statusword_t;

typedef struct DSP402_CONTROLWORD_BITS
{
    uint16_t switch_on:1;
    uint16_t enable_voltage:1;
    uint16_t quick_stop:1;
    uint16_t enable_operation:1;
    uint16_t operation_mode_specific_0:3;
    uint16_t fault_reset:1;
    uint16_t halt:1;
    uint16_t operation_mode_specific_1:1;
    uint16_t rsrvd:1;
    uint16_t manufacturer_specific:5;

} dsp402_controlword_bits_t;

typedef union DSP402_CONTROLWORD_TYPE
{
    uint16_t all;
    dsp402_controlword_bits_t bits;

} dsp402_controlword_t;


void ecatcomm_slave_print_statusword(dsp402_statusword_t statusword)
{
    // Printout
    printf("\n\n");
    printf("statusword.ready_to_switch_on         = %d\n", statusword.bits.ready_to_switch_on);
    printf("statusword.switched_on                = %d\n", statusword.bits.switched_on);
    printf("statusword.operation_enabled          = %d\n", statusword.bits.operation_enabled);
    printf("statusword.fault                      = %d\n", statusword.bits.fault);
    printf("statusword.volt_enabled               = %d\n", statusword.bits.voltage_enabled);
    printf("statusword.quick_stop                 = %d\n", statusword.bits.quick_stop);
    printf("statusword.switch_on_disabled         = %d\n", statusword.bits.switch_on_disabled);
    printf("statusword.warning                    = %d\n", statusword.bits.warning);
    printf("statusword.manufacturer_specific      = %d\n", statusword.bits.manufacturer_specific_0);
    printf("statusword.remote                     = %d\n", statusword.bits.remote);
    printf("statusword.operation_mode_specific_0  = %d\n", statusword.bits.operation_mode_specific_0);
    printf("statusword.internal_limit_active      = %d\n", statusword.bits.internal_limit_active);
    printf("statusword.operation_mode_specific_1  = %d\n", statusword.bits.operation_mode_specific_1);
    printf("statusword.operation_mode_specific_2  = %d\n", statusword.bits.operation_mode_specific_2);
    printf("statusword.manufacturer_specific      = %d\n", statusword.bits.manufacturer_specific_1);
}

void ecatcomm_slave_print_controlword(dsp402_controlword_t controlword)
{
    // Printout
    printf("\n\n");
    printf("controlword.switch_on                 = %d\n", controlword.bits.switch_on);
    printf("controlword.enable_voltage            = %d\n", controlword.bits.enable_voltage);
    printf("controlword.quick_stop                = %d\n", controlword.bits.quick_stop);
    printf("controlword.enable_operation          = %d\n", controlword.bits.enable_operation);
    printf("controlword.operation_mode_specific_0 = %d\n", controlword.bits.operation_mode_specific_0);
    printf("controlword.fault_reset               = %d\n", controlword.bits.fault_reset);
    printf("controlword.halt                      = %d\n", controlword.bits.halt);
    printf("controlword.operation_mode_specific_1 = %d\n", controlword.bits.operation_mode_specific_1);
    printf("controlword.manufacturer_specific     = %d\n", controlword.bits.manufacturer_specific);
}



typedef struct ELMO_INDATA_TYPE
{
    dsp402_statusword_t statusword;

    int position;
    int velocity;

    int digitalin;

    int busvoltage;
    int motorcurrent;

} elmo_twitter_indata_t;

typedef struct ELMO_OUTDATA_TYPE
{
    dsp402_controlword_t controlword;

    int torque;

} elmo_twitter_outdata_t;




void ecatcomm_slave_set_rxpdo(elmo_twitter_outdata_t *pdata, int command, double torque)
{
    static dsp402_controlword_t controlword = {0};
    int16_t torque_data=0;

    // Set controlword data
    switch(command)
    {
        case SWITCH_ON:
            controlword.bits.switch_on = 1;
            break;

        case SHUTDOWN:
            controlword.bits.switch_on = 0;
            break;

        case DISABLE_VOLTAGE:
            controlword.bits.enable_voltage = 0;
            break;

        case ENABLE_VOLTAGE:
            controlword.bits.enable_voltage = 1;
            break;

        case QUICK_STOP:
            controlword.bits.quick_stop = 1;
            break;

        case DISABLE_OPERATION:
            controlword.bits.enable_operation = 0;
            break;

        case ENABLE_OPERATION:
            controlword.bits.enable_operation = 1;
            break;

        case FAULT_RESET:
            controlword.bits.fault_reset = 1;
            break;

        case HALT:
            controlword.bits.halt = 1;
            break;

        case HALT_RESET:
            controlword.bits.halt = 0;
            break;

        case CLEAR_CONTROLWORD:
            controlword.all = 0;
            break;
    }

    // Covert torque data to INT16 from double
    double rated_current = 20000.0;
    double rated_torque = (20000.0*0.27)*0.001;
    torque_data = (int16_t)(torque/rated_torque*1000.0);

    // Copy to internal struct
    pdata->controlword.all = controlword.all;
    pdata->torque = torque_data;
}

void ecatcomm_slave_get_txpdo(elmo_twitter_indata_t *pdata, const tcan::EtherCatDatagram& datagram)
{
    uint16 statusword=0;
    char databuffer[21];

    // Get data
    memcpy(&databuffer[0], datagram.getData(), datagram.getDataLength());

    // Store data
    pdata->statusword.all = ((databuffer[13] << 8 ) & 0xff00) | (databuffer[12] & 0xff);
    pdata->position = ((databuffer[3] << 24) & 0xff000000) | ((databuffer[2] << 16) & 0x00ff0000) | ((databuffer[1] << 8) & 0x0000ff00) | ((databuffer[0] << 0) & 0x000000ff);
    pdata->digitalin = ((databuffer[7] << 24) & 0xff000000) | ((databuffer[6] << 16) & 0x00ff0000) | ((databuffer[5] << 8) & 0x0000ff00) | ((databuffer[4] << 0) & 0x000000ff);
    pdata->velocity = ((databuffer[11] << 24) & 0xff000000) | ((databuffer[10] << 16) & 0x00ff0000) | ((databuffer[9] << 8) & 0x0000ff00) | ((databuffer[8] << 0) & 0x000000ff);
    pdata->busvoltage = ((databuffer[18] << 24) & 0xff000000) | ((databuffer[17] << 16) & 0x00ff0000) | ((databuffer[16] << 8) & 0x0000ff00) | ((databuffer[15] << 0) & 0x000000ff);
    pdata->motorcurrent = ((databuffer[20] << 8) & 0xff00) | ((databuffer[19] << 0) & 0x00ff);
}

elmo_twitter_outdata_t outdata_;
elmo_twitter_indata_t indata_;


tcan::EtherCatDatagrams createDatagrams() {

  static int test_step=0;
  static int step_counter=0;

  // Test procedure
  switch (test_step)
  {
      case 0: // Reset errors
          if (step_counter >= 2000)
          {
              step_counter = 0;
              test_step++;
          }
          else if (step_counter == 1)
          {
              ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, FAULT_RESET, 0.0);
              ecatcomm_slave_print_controlword(outdata_.controlword);
              ecatcomm_slave_print_statusword(indata_.statusword);
              printf("Clearing errors...\n\n");
          }
          break;

      case 1: // Startup
          if (step_counter >= 5000)
          {
              step_counter = 0;
              test_step++;
          }
          else if (step_counter == 1)
          {
              ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
              ecatcomm_slave_print_controlword(outdata_.controlword);
              ecatcomm_slave_print_statusword(indata_.statusword);
              printf("Startup up drive...\n\n");
          }
          break;

      case 2: // Switch on
          if (step_counter >= 5000)
          {
              step_counter = 0;
              test_step++;
          }
          else if (step_counter == 1)
          {
              ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, SWITCH_ON, 0.0);
              ecatcomm_slave_print_controlword(outdata_.controlword);
              ecatcomm_slave_print_statusword(indata_.statusword);
              printf("Switching on drive...\n\n");
          }
          break;
      case 3: // enable operation
          if(step_counter >= 5000)
          {
              step_counter = 0;
              test_step++;
          }
          else if (step_counter == 1)
          {
              ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, SWITCH_ON, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_OPERATION, 0.0);
              ecatcomm_slave_print_controlword(outdata_.controlword);
              ecatcomm_slave_print_statusword(indata_.statusword);
              printf("Enabling operation...\n\n");
          }
      break;
      case 4: // wait delay
          if(step_counter >= 7000)
          {
              step_counter = 0;
              test_step++;
          }
          else if (step_counter == 1)
          {
              ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, SWITCH_ON, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_OPERATION, 0.0);
              ecatcomm_slave_print_controlword(outdata_.controlword);
              ecatcomm_slave_print_statusword(indata_.statusword);
              printf("Running test...\n\n");
          }
          break;
      case 5: // Run test output
          if(step_counter >= 7000)
          {
              step_counter = 0;
              test_step++;
          }
          else if (step_counter == 1)
          {
              ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, SWITCH_ON, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_OPERATION, 0.0);
              ecatcomm_slave_print_controlword(outdata_.controlword);
              ecatcomm_slave_print_statusword(indata_.statusword);
              printf("Running test...\n\n");
          }
          break;
      case 6: // Stop before end
          if(step_counter >= 5000)
          {
              step_counter = 0;
              test_step++;
          }
          else if (step_counter == 1)
          {
              ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, SWITCH_ON, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_OPERATION, 0.0);
              ecatcomm_slave_print_controlword(outdata_.controlword);
              ecatcomm_slave_print_statusword(indata_.statusword);
              printf("Stopping test...\n\n");
          }
          break;
      case 7: // Stop before end
          if(step_counter >= 5000)
          {
              step_counter = 0;
              test_step++;
          }
          else if (step_counter == 1)
          {
              ecatcomm_slave_set_rxpdo(&outdata_, CLEAR_CONTROLWORD, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, SWITCH_ON, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, ENABLE_VOLTAGE, 0.0);
              ecatcomm_slave_set_rxpdo(&outdata_, QUICK_STOP, 0.0);
              ecatcomm_slave_print_controlword(outdata_.controlword);
              ecatcomm_slave_print_statusword(indata_.statusword);
              printf("Exiting test...\n\n");
          }
          break;
  }

  step_counter++;



  // Write to output buffer
  char databuffer[4];
  databuffer[0] = ((outdata_.controlword.all >> 0) & 0xff);
  databuffer[1] = ((outdata_.controlword.all >> 8) & 0xff);
  databuffer[2] = ((outdata_.torque >> 0) & 0xff);
  databuffer[3] = ((outdata_.torque >> 8) & 0xff);

  tcan::EtherCatDatagrams datagrams;
  tcan::EtherCatDatagram rxDatagram;
  rxDatagram.resize(4);
  tcan::EtherCatDatagram txDatagram;
  txDatagram.resize(21);
  memcpy(rxDatagram.data_, &databuffer[0], 4);
  datagrams.rxAndTxDatagrams_.insert({1, {rxDatagram, txDatagram}});
  return datagrams;
}




bool g_running = true;

void signal_handler(int) {
	g_running = false;
}

int main() {
  const bool asynchronous = false;

	signal(SIGINT, signal_handler);

  tcan::EtherCatDevice device(1, "? M:0000009a I:00030924");

	tcan::EtherCatBusOptions* busOptions = new tcan::EtherCatBusOptions(); // TODO: Why are the options destroyed in the bus destructor?
	busOptions->name_ = "enp0s31f6";
	busOptions->asynchronous_ = asynchronous;
	tcan::EtherCatBus bus(busOptions);
  bus.addDevice(&device);

  tcan::EtherCatBusManager busManager;
	if (!busManager.addBus(&bus)) {
	  MELO_ERROR_STREAM("Bus could not be added.");
	  return 0;
	}
//  std::cout << "bus added and initialized" << std::endl;

	auto nextStep = std::chrono::steady_clock::now();

	while(g_running) {
//	  std::cout << "in loop" << std::endl;
    if (!asynchronous) {
      busManager.readMessagesSynchrounous();
      busManager.sanityCheckSynchronous();
    }


    static int i = 0;
    static int print_counter=0;

    if(++print_counter >= 5) {
//        printf("Processdata cycle %4d, WKC %d", i++, wkc_.load());
        printf("Processdata cycle %4d", i++);

//        printf(", Outputs:");
//        for(int j = 0 ; j < oloop_; j++)
//        {
//            printf(" %2.2x", *(ecatContext_.slavelist[0].outputs + j));
//        }
        // printf(", Inputs:");
        // for(j = 0 ; j < iloop; j++)
        // {
        //     printf(" %2.2x", *(ecatContext_.slavelist[0].inputs + j));
        // }

        // printf(" T:%"PRId64"",ecatContext_.DCtime[0]);
        printf(", Command Data: 0x%4x, %4d", outdata_.controlword.all, outdata_.torque);
        printf(", Feedback Data: 0x%4x, %8d, %8d, %8d, %8d", indata_.statusword.all, indata_.position, indata_.velocity, indata_.busvoltage, indata_.motorcurrent);
        printf("\r");
        print_counter = 0;
    }



    if (bus.getData())
        ecatcomm_slave_get_txpdo(&indata_, bus.getData()->rxAndTxDatagrams_[1].second);
	  bus.emplaceMessage(createDatagrams());



		// as an alternative, sendMessage(..) can be used, if emplacing the message is not appropriate
    if (!asynchronous) {
//      std::cout << " writeMessagesSynchronous " << std::endl;
      busManager.writeMessagesSynchronous();
    }

		nextStep += std::chrono::microseconds(1000);
		std::this_thread::sleep_until( nextStep );
	}
	return 0;
}
