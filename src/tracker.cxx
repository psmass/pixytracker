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
  dds::core::Duration DEFAULT_PERIOD {0,100000000}; // 100 ms default HB writer rate
  #define TEN_SEC 40 // main loop clock tick is 250ms. 40 = ten sec
  
  enum SM_States {INITIALIZE, VOTE, WAIT_VOTES_IN, VOTE_RESULTS, STEADY_STATE, SHUT_DOWN, ERROR};
  
void run_tracker_application(unsigned int tracked_channel) {
   // Create the participant
    dds::core::QosProvider qos_provider({ MODULE::QOS_FILE });
    dds::domain::DomainParticipant participant =
      qos_provider->create_participant_from_config("PixyTrackerParticipant_Library::PixyTrackerParticipant");

    RedundancyInfo redundancy_info(participant); // info to share beteween Heartbeat and Vote logic
    
    // Instantiate Topic Readers and Writers w/threads. Note, the names eg. servo_writer is
    // just a handle to the server_writer and not the actual DDS writer (use the getMyDataWriter())
    // fucntion from the ddsEntities.hpp writer class.
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
    int cycle_cnt {5}; // Used to slow state printouts, Print first state entry

    // used in SM to change ownership strengthor servo_writer  based on my roll vote
    dds::pub::qos::DataWriterQos writer_qos = servo_writer.getMyDataWriter().qos();
    dds::core::policy::OwnershipStrength ownership_strength;
    int ownership_strength_value {1};

    
    while (!application::shutdown_requested) {
      //
      // This block describes a state machine implemented int the main thread
      // (here below) that will have the following states:
      // INITIALIZE, VOTE, WAIT_FOR_VOTES, VOTE_RESULTS, STEADY_STATE, SHUT_DOWN 
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
	// Note:  The sleep in INITIALIZE ensures we wait at least 1 sec after
	//        all the trackers, if all three trackers are up, and makes a one time
	//        worst case 10sec startup.
	//        Any delay in WAIT_VOTES_IN either only adds a one time delay to
	//        initialization, or if operational, determines how fast the a
	//        reassessment is while the PRIMARY or SECONDARY continues to
	//        publish (i.e. both cases simply time to STEADY_STATE) - i.e.
	//        HB dead-line determines switch over time an not reVoteing.
	//       
      case INITIALIZE:
	// we stay here waiting for up to 3 trackers or upto 10 seconds
	if (!(cycle_cnt++ %5)) {
	  cycle_cnt=1;
	  std::cout << ":" << std::flush;
	}
	if (redundancy_info.numberOfTrackers()==3 || ten_sec_cnt==TEN_SEC) {
	  rti::util::sleep(dds::core::Duration(1)); // extra sec to register HBs
	  state=VOTE;
	};
	break;
	
      case VOTE:
	// this state tranitions quickly once we vote and ensure one vote
	std::cout << "\nSTATE: VOTING" << std::endl;
	vote_wtr.vote(); // place my vote for Primary/Sec/Tertiary
	state=WAIT_VOTES_IN;
	cycle_cnt=5; // make sure we print the first entry to each state
	break;

      case WAIT_VOTES_IN:
	std::cout << "Votes In = " << redundancy_info.votesIn()
		  << " Trackers =: " << redundancy_info.numberOfTrackers()
		  << std::endl;

	if (!(cycle_cnt++ %5)) {
	  cycle_cnt = 1;
	  std::cout << "\nSTATE: WAITING FOR ALL VOTES" << std::endl;
	};
	// wait for all votes to be in, if < 3 the timing is dependent
	// upon delays from different trackers starting.
        if (redundancy_info.votesIn() == redundancy_info.numberOfTrackers()) 
	  state=VOTE_RESULTS;
	break;

      case VOTE_RESULTS: 
	std::cout << "\nSTATE: ASSESSING VOTING RESULTS" << std::endl;
	// Assess if all trackers are consistent - fault any missing
	// or inconsistent trackers. Set Pixy_Servo_Strength based on
	// results: PRIMARY 30, SECONDARY 20, TERTIARY 10
	redundancy_info.assessVoteResults();
	// clear voting immediatly after tally for next potential vote
	// not in vote state, since votes are durable and may already
	// be in upon restart.
	redundancy_info.clearVotes(); 
	redundancy_info.printSortedTrackers();

	// change my ownership strength based on my roll.
        ownership_strength_value = redundancy_info.getMyRollStrength();
	ownership_strength.value(ownership_strength_value);
	writer_qos << ownership_strength;
	servo_writer.getMyDataWriter().qos(writer_qos);
	servo_writer.enable(); // enable after we've set the strength

	//redundancy_info.printVoteResults();
	state=STEADY_STATE;
	break;
	
      case STEADY_STATE:
	servo_writer.printGimbalPosition();
        //std::cout << "." << std::flush;
	// check for hb deadline missed from a tracker. The SM runs at 250ms
	// the HB run at 100ms. The SM will clear the count, receive HBs will
	// increment the count so a 0 will indicate a deadline miss.
	// note: ignore our own count since we don't get our own HBs
	for (int i=0; i<redundancy_info.numberOfTrackers(); i++){
	  if ((i !=redundancy_info.getMyOrdinal()-1) && \
	      (redundancy_info.getTrackerState_ptr(i)->hbDeadlineCnt == 0)) {
	    std::cout << "\nFAULT DEADLINE MISS -  Tracker: "
		      << redundancy_info.getTrackerState_ptr(i)->guid
		      << std::endl;
	    // we need to drop this tracker and promote all lower trackers
	    redundancy_info.lostTracker(i);
	    state=VOTE;
	    break;
	  } else {
	  // zero the count
	  redundancy_info.getTrackerState_ptr(i)->hbDeadlineCnt = 0;
	  }
	} // for
	
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
      rti::util::sleep(dds::core::Duration(0,250000000));
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

