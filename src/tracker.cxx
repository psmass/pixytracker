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

  enum SM_States {INITIALIZE, VOTE, STEADY_STATE, SHUT_DOWN, ERROR};
  
void run_tracker_application(unsigned int tracked_channel) {
   // Create the participant
    dds::core::QosProvider qos_provider({ MODULE::QOS_FILE });
    dds::domain::DomainParticipant participant =
      qos_provider->create_participant_from_config("PixyTrackerParticipant_Library::PixyTrackerParticipant");

    RedundancyInfo redundancy_info(participant); // info to share beteween Heartbeat and Vote logic
    
    // Instantiate Topic Readers and Writers w/threads
    ServoWtr servo_writer(participant); 
    ShapesRdr shapes_reader(participant, &servo_writer);
    HeartbeatWtr tracker_hb_wtr(participant, &redundancy_info, PERIODIC, DEFAULT_PERIOD);
    HeartbeatRdr tracker_hb_rdr(participant, &redundancy_info);
    VoteWtr vote_wtr(participant, &redundancy_info);
    VoteRdr vote_rdr(participant, &redundancy_info);
    
    // Ignore our own writers (e.g. heartbeat and votes
    dds::domain::ignore(participant, participant.instance_handle());

    shapes_reader.runThread();
    tracker_hb_rdr.runThread();
    tracker_hb_wtr.runThread();
    vote_rdr.runThread();

    // *** START WRITER LISTENERS or MONITOR THREADS (This step Optional)
    //servo_writer.runThread() # start a statuses monitor thread on the DA Writer
    //or...#listener, Heartbeat is periodic and will run as a thread
    DefaultWriterListener * listener = new DefaultWriterListener; 
    servo_writer.getMyDataWriter().listener (listener, dds::core::status::StatusMask::all());

    enum SM_States state = INITIALIZE;
    int ten_sec_cnt {0};    // initial worst case wait period to vote
    
    while (!application::shutdown_requested) {
      //
      // This block describes a state machine implemented int the main thread
      // (here below) that will have the following states:
      // INITIALIZE, VOTE, STEADY_STATE, SHUT_DOWN 
      //
      // The state machine will INITIALIZE to wait 10 sec for 3 trackers or
      // 3 trackers which ever comes first and then proceed to VOTE state.
      //
      // In the VOTE State, voting should occur as follows:
      // (1) An operating system should not cause a primary or secondary change
      // this means that Late Joiners (i.e. they receive a durable Vote topic
      // while in the initialize phase should vote first for any current PRIMARY
      // and SECONDARY unit placeing themselve in as the next available
      // secondary or tertiary unit.
      // (2) If they are not a late joiner as per above, they must then vote
      // based on ordinal (lowest to highest). Any unit receiving two votes
      // for PRIMARY, SECONDARY, or TERTIARY must take that role, adjusting
      // the servo_control topic strength accordingly. Once a vote has been
      // writen, the SM will transition to STEADY_STATE
      //
      // In the STEADY_STATE the SM will maintain the LEDS / print out to
      // reflect it's roll (PRIMARY, SECONDARY, TERITARY), and Ordinal(1,2,3).
      // NOTE: Roll and Ordinal are not the same.
      // It will also reflect the all trackers GUIDs rolls, and ordinals and
      // report any failed or missing trackers.
      // From STEADY_STATE a unit can reenter the VOTE state upon change of
      // detected trackers (i.e. loss of heartbeat or new tracker heartbeat),
      // or it may SHUT_DOWN if directed.
      //

      switch (state) {

      case INITIALIZE:
	// we stay here waiting for up to 3 trackers or upto 10 seconds
	std::cout << "i." << redundancy_info.numberOfTrackers() << std::flush;
	if (redundancy_info.numberOfTrackers()==3 || ten_sec_cnt==10)
	  state=VOTE;
	break;
	
      case VOTE:
	// this state tranitions quickly once we vote
	std::cout << "\n STATE: VOTING" << std::endl;
	redundancy_info.assessVote(); // place my vote for Primary/Sec/Tertiary
	state=STEADY_STATE;
	break;
	
      case STEADY_STATE:
	std::cout << "." << std::flush;
	break;
	
      case SHUT_DOWN:
	break;
	
      case ERROR: // detectable error state
	// print message/red light LEDs and SHUT DOWN
	state=SHUT_DOWN;
	break;
      default:
	;
      } // switch
      
      rti::util::sleep(dds::core::Duration(1));
      ten_sec_cnt++;
    }

    vote_rdr.Reader::getThreadHndl()->join();
    shapes_reader.Reader::getThreadHndl()->join();
    tracker_hb_rdr.Reader::getThreadHndl()->join();
    tracker_hb_wtr.Writer::getThreadHndl()->join();
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

