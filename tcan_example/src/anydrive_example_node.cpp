#include <signal.h>
#include <unordered_map>

#include "tcan/EtherCatBus.hpp"
#include "tcan/EtherCatBusManager.hpp"

#include "tcan_example/Anydrive.hpp"


using namespace tcan_example;


tcan::EtherCatDatagrams createDatagrams(AnydriveOutdata& outdata) {

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
                outdata = createOutdata(Dsp402Command::CLEAR_CONTROLWORD, 0.0);
                outdata = createOutdata(Dsp402Command::FAULT_RESET, 0.0);
                printControlword(outdata.controlword);
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
                outdata = createOutdata(Dsp402Command::CLEAR_CONTROLWORD, 0.0);
                outdata = createOutdata(Dsp402Command::ENABLE_VOLTAGE, 0.0);
                outdata = createOutdata(Dsp402Command::QUICK_STOP, 0.0);
                printControlword(outdata.controlword);
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
                outdata = createOutdata(Dsp402Command::CLEAR_CONTROLWORD, 0.0);
                outdata = createOutdata(Dsp402Command::ENABLE_VOLTAGE, 0.0);
                outdata = createOutdata(Dsp402Command::QUICK_STOP, 0.0);
                outdata = createOutdata(Dsp402Command::SWITCH_ON, 0.0);
                printControlword(outdata.controlword);
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
                outdata = createOutdata(Dsp402Command::CLEAR_CONTROLWORD, 0.0);
                outdata = createOutdata(Dsp402Command::SWITCH_ON, 0.0);
                outdata = createOutdata(Dsp402Command::ENABLE_VOLTAGE, 0.0);
                outdata = createOutdata(Dsp402Command::QUICK_STOP, 0.0);
                outdata = createOutdata(Dsp402Command::ENABLE_OPERATION, 0.0);
                printControlword(outdata.controlword);
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
                outdata = createOutdata(Dsp402Command::CLEAR_CONTROLWORD, 0.0);
                outdata = createOutdata(Dsp402Command::SWITCH_ON, 0.0);
                outdata = createOutdata(Dsp402Command::ENABLE_VOLTAGE, 0.0);
                outdata = createOutdata(Dsp402Command::QUICK_STOP, 0.0);
                outdata = createOutdata(Dsp402Command::ENABLE_OPERATION, 0.0);
                printControlword(outdata.controlword);
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
                outdata = createOutdata(Dsp402Command::CLEAR_CONTROLWORD, 0.0);
                outdata = createOutdata(Dsp402Command::SWITCH_ON, 0.0);
                outdata = createOutdata(Dsp402Command::ENABLE_VOLTAGE, 0.0);
                outdata = createOutdata(Dsp402Command::QUICK_STOP, 0.0);
                outdata = createOutdata(Dsp402Command::ENABLE_OPERATION, 0.0);
                printControlword(outdata.controlword);
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
                outdata = createOutdata(Dsp402Command::CLEAR_CONTROLWORD, 0.0);
                outdata = createOutdata(Dsp402Command::SWITCH_ON, 0.0);
                outdata = createOutdata(Dsp402Command::ENABLE_VOLTAGE, 0.0);
                outdata = createOutdata(Dsp402Command::QUICK_STOP, 0.0);
                outdata = createOutdata(Dsp402Command::ENABLE_OPERATION, 0.0);
                printControlword(outdata.controlword);
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
                outdata = createOutdata(Dsp402Command::CLEAR_CONTROLWORD, 0.0);
                outdata = createOutdata(Dsp402Command::SWITCH_ON, 0.0);
                outdata = createOutdata(Dsp402Command::ENABLE_VOLTAGE, 0.0);
                outdata = createOutdata(Dsp402Command::QUICK_STOP, 0.0);
                printf("Exiting test...\n\n");
            }
            break;
    }

    step_counter++;

    // Write to output buffer
    uint8_t databuffer[32];
    for (unsigned int i = 0; i < 32; i++)
      databuffer[i] = 0;
    databuffer[0] = ((outdata.controlword.all >> 0) & 0xff);
    databuffer[1] = ((outdata.controlword.all >> 8) & 0xff);

    tcan::EtherCatDatagrams datagrams;
    tcan::EtherCatDatagram rxDatagram;
    rxDatagram.resize(32);
    rxDatagram.setZero();
    tcan::EtherCatDatagram txDatagram;
    txDatagram.resize(56);
    txDatagram.setZero();
    memcpy(rxDatagram.data_, &databuffer[0], 4);
    datagrams.rxAndTxDatagrams_.insert({1, {rxDatagram, txDatagram}});
    return datagrams;
}




bool g_running = true;

void signal_handler(int signal) {
    g_running = false;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        MELO_ERROR_STREAM("Missing port name (e.g. enp0s31f6).");
        return 0;
    }

    const bool asynchronous = false;

    signal(SIGINT, signal_handler);

    Anydrive device(1, "ANYdrive");

    tcan::EtherCatBusOptions* busOptions = new tcan::EtherCatBusOptions(); // TODO: Why are the options destroyed in the bus destructor?
    busOptions->name_ = argv[1];
    busOptions->asynchronous_ = asynchronous;
    tcan::EtherCatBus bus(busOptions);
    bus.addDevice(&device);

    tcan::EtherCatBusManager busManager;
    if (!busManager.addBus(&bus)) {
        MELO_ERROR_STREAM("Bus could not be added.");
        return 0;
    }
  //  std::cout << "bus added and initialized" << std::endl;





    bus.checkSlaveStates();
    device.configureSlave();
    bus.checkSlaveStates();
    bus.checkSlaveErrors();


    bus.strangeFunction();









    auto nextStep = std::chrono::steady_clock::now();

    int i = 0;
    int print_counter = 0;
    int print_counter_statusword = 0;

    std::cout << "STARTING LOOP" << std::endl;

    while(g_running) {
        if (!asynchronous) {
            busManager.readMessagesSynchrounous();
            busManager.sanityCheckSynchronous();
        }

        AnydriveOutdata outdata;
        AnydriveIndata indata;

        bus.emplaceMessage(createDatagrams(outdata));

        if (bus.getData()) {
            indata = createIndata(bus.getData()->rxAndTxDatagrams_[1].second);

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
                printf(", Command Data: 0x%4x, %4d", outdata.controlword.all, outdata.desired_joint_position);
                printf(", Feedback Data: 0x%4x, %8d, %8d, %8d, %8d, %8d", indata.statusword.all, indata.measured_motor_voltage, indata.measured_motor_current, indata.measured_motor_position, indata.measured_gear_position, indata.measured_joint_position);
                printf("\r");
                print_counter = 0;
            }
        }

        if (++print_counter_statusword >= 100) {
            printStatusword(indata.statusword);
            print_counter_statusword = 0;
        }

    // as an alternative, sendMessage(..) can be used, if emplacing the message is not appropriate
        if (!asynchronous) {
            busManager.writeMessagesSynchronous();
        }

        nextStep += std::chrono::microseconds(1000);
        std::this_thread::sleep_until( nextStep );
    }
    return 0;
}
