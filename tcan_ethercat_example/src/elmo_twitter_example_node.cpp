#include <signal.h>
#include <unordered_map>

#include "tcan_ethercat/EtherCatBus.hpp"
#include "tcan_ethercat/EtherCatBusManager.hpp"

#include "tcan_ethercat_example/ElmoTwitter.hpp"


using namespace tcan_ethercat_example;


tcan_ethercat::EtherCatDatagrams createDatagrams(ElmoTwitterOutdata& outdata) {

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
    char databuffer[4];
    databuffer[0] = ((outdata.controlword.all >> 0) & 0xff);
    databuffer[1] = ((outdata.controlword.all >> 8) & 0xff);
    databuffer[2] = ((outdata.torque >> 0) & 0xff);
    databuffer[3] = ((outdata.torque >> 8) & 0xff);

    tcan_ethercat::EtherCatDatagrams datagrams;
    tcan_ethercat::EtherCatDatagram rxDatagram;
    rxDatagram.resize(4);
    rxDatagram.setZero();
    tcan_ethercat::EtherCatDatagram txDatagram;
    txDatagram.resize(21);
    txDatagram.setZero();
    memcpy(rxDatagram.data_, &databuffer[0], 4);
    datagrams.rxAndTxPdoDatagrams_.insert({1, {rxDatagram, txDatagram}});
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

    const tcan::BusOptions::Mode mode = tcan::BusOptions::Mode::Asynchronous;

    signal(SIGINT, signal_handler);

    ElmoTwitter slave(1, "? M:0000009a I:00030924");

    std::unique_ptr<tcan_ethercat::EtherCatBusOptions> busOptions(new tcan_ethercat::EtherCatBusOptions());
    busOptions->name_ = argv[1];
    busOptions->mode_ = mode;
    tcan_ethercat::EtherCatBus bus(std::move(busOptions));
    bus.addSlave(&slave);

    tcan_ethercat::EtherCatBusManager busManager;
    if (!busManager.addBus(&bus)) {
        MELO_ERROR_STREAM("Bus could not be added.");
        return 0;
    }
    if (!bus.setupCommunication()) {
        MELO_ERROR_STREAM("Bus communication could not be setup.");
        return 0;
    }

    auto nextStep = std::chrono::steady_clock::now();

    int i = 0;
    int print_counter = 0;
    int print_counter_statusword = 0;

    while(g_running) {
        if (mode != tcan::BusOptions::Mode::Synchronous) {
            busManager.readMessagesSynchronous();
            busManager.sanityCheckSynchronous();
        }

        ElmoTwitterOutdata outdata;
        ElmoTwitterIndata indata;

        // as an alternative, sendMessage(..) can be used, if emplacing the message is not appropriate
        bus.emplaceMessage(createDatagrams(outdata));

        if (bus.getData()) {
            indata = createIndata(bus.getData()->rxAndTxPdoDatagrams_[1].second);

            if(++print_counter >= 5) {
                printf("Processdata cycle %4d", i++);
                // printf(" T:%"PRId64"",ecatContext_.DCtime[0]);
                printf(", Command Data: 0x%4x, %4d", outdata.controlword.all, outdata.torque);
                printf(", Feedback Data: 0x%4x, %8d, %8d, %8d, %8d", indata.statusword.all, indata.position, indata.velocity, indata.busvoltage, indata.motorcurrent);
                printf("\r");
                print_counter = 0;
            }
        }

        if (++print_counter_statusword >= 100) {
            printStatusword(indata.statusword);
            print_counter_statusword = 0;
        }

        if (mode != tcan::BusOptions::Mode::Asynchronous) {
            busManager.writeMessagesSynchronous();
        }

        nextStep += std::chrono::microseconds(1000);
        std::this_thread::sleep_until(nextStep);
    }
    return 0;
}
