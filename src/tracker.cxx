/*
 * (c) Copyright, Real-Time Innovations, 2022.  All rights reserved.
 * RTI grants Licensee a license to use, modify, compile, and create derivative
 * works of the software solely for use with RTI Connext DDS. Licensee may
 * redistribute copies of the software provided that all such copies are subject
 * to this license. The software is provided "as is", with no warranty of any
 * type, including any warranty for fitness for any purpose. RTI is under no
 * obligation to maintain or support the software. RTI shall not be liable for
 * any incidental or consequential damages arising out of the use or inability
 * to use the software.
 */

#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>
#include <dds/dds.hpp>
#include <rti/util/util.hpp> // for sleep
#include "gimbal.hpp"
#include "tracker.hpp"   // rti generated file from idl to use model const Topics
#include "ddsEntities.hpp"
#include "topics.hpp"
#include "application.hpp"

namespace MODULE
{
  #define PERIODIC true
  dds::core::Duration DEFAULT_PERIOD {1,0}; // 1 second default writer rate
  
void run_tracker_application(unsigned int tracked_channel) {
   // Create the participant
    dds::core::QosProvider qos_provider({ MODULE::QOS_FILE });
    dds::domain::DomainParticipant participant =
      qos_provider->create_participant_from_config("PixyTrackerParticipant_Library::PixyTrackerParticipant");


    // Instantiate Topic Readers and Writers w/threads
    ServoWtr servo_writer(participant); 
    ShapesRdr shapes_reader(participant, &servo_writer);
    HeartbeatWtr controller_hb_wtr(participant, PERIODIC, DEFAULT_PERIOD);
    HeartbeatRdr controller_hb_rdr(participant);

    shapes_reader.runThread();
    controller_hb_rdr.runThread();
    controller_hb_wtr.runThread();

    // *** START WRITER LISTENERS or MONITOR THREADS (This step Optional)
    //servo_writer.runThread() # start a statuses monitor thread on the DA Writer
    //or...#listener, Heartbeat is periodic and will run as a thread
    DefaultWriterListener * listener = new DefaultWriterListener; 
    servo_writer.getMyDataWriter().listener (listener, dds::core::status::StatusMask::all());


    while (!application::shutdown_requested) {
        // topic threads are running receiving data and writing servo control
        // look for three-way voting (perhaps control LEDs here)
        std::cout << "." << std::flush;                 
        rti::util::sleep(dds::core::Duration(1));
    }

   
    shapes_reader.Reader::getThreadHndl()->join();
    controller_hb_rdr.Reader::getThreadHndl()->join();
    controller_hb_wtr.Writer::getThreadHndl()->join();
    // give threads a second to shut down
    rti::util::sleep(dds::core::Duration(1));
    std::cout << "Tracker main thread shutting down" << std::endl;

}
} // namespace MODULE

int main(int argc, char *argv[]) {

  const char *sigName[] = {
    "RED",
    "ORANGE",
    "YELLOW",
    "GREEN",
    "CYAN",
    "BLUE",
    "PURPLE"
   };

    using namespace application;
    unsigned int trackedChannel = INDEX_YELLOW;

    setup_signal_handlers();

    printf("PIXY TRACKER: %s %s\n", __DATE__, __TIME__);
    //printf("DomainID: %d\n", domainId);

    if (argc > 1) {
        for (int count = 1; count < argc; count++) {
	  for (int sigs = 0; sigs < NUM_SIGS; sigs++) {
	    if (strcmp(argv[count], sigName[sigs])== 0) {
                    trackedChannel = sigs;
                    break;
                }
            }
        }
    }

    if (trackedChannel > NUM_SIGS) trackedChannel = INDEX_GREEN;
    printf("Tracking %s\n", sigName[trackedChannel]);
    
    try {
        MODULE::run_tracker_application(trackedChannel);
    }
    catch (const std::exception &ex) {
        // This will catch DDS exceptions
        std::cerr << "Exception in run_controller_application(): " << ex.what()
                  << std::endl;
        return EXIT_FAILURE;
    }

    // Releases the memory used by the participant factory.  Optional at
    // application exit
    // dds::domain::DomainParticipant::finalize_participant_factory();

    return EXIT_SUCCESS;
}

