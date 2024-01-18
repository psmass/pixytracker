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
  dds::core::Duration DEFAULT_PERIOD  {0,250000000}; // 250 ms default HB writer rate
  dds::core::Duration MAINLOOP_PERIOD {1,0}; // 1 sec mainloop ensures 2 hbs per
  
void run_tracker_application(unsigned int tracked_channel) {
   // Create the participant
    dds::core::QosProvider qos_provider({ MODULE::QOS_FILE });
    dds::domain::DomainParticipant participant =
      qos_provider->create_participant_from_config("PixyTrackerParticipant_Library::PixyTrackerParticipant");

    RedundancyDb redundancy_db(participant); // db to share beteween Heartbeat and Vote logic
    
    // Instantiate Topic Readers and Writers w/threads. Note, the names eg. servo_writer is
    // just a handle to the server_writer and not the actual DDS writer (use the getMyDataWriter())
    // fucntion from the ddsEntities.hpp writer class.
    ServoWtr servo_writer(participant); 
    ShapesRdr shapes_reader(participant, &servo_writer);
    HeartbeatWtr tracker_hb_wtr(participant, &redundancy_db, PERIODIC, DEFAULT_PERIOD);
    HeartbeatRdr tracker_hb_rdr(participant, &redundancy_db);
    VoteWtr vote_wtr(participant, &redundancy_db);
    VoteRdr vote_rdr(participant, &redundancy_db);

    
    // Ignore our own writers (e.g. heartbeat and votes
    dds::domain::ignore(participant, participant.instance_handle());

    shapes_reader.runThread();
    tracker_hb_rdr.runThread();
    tracker_hb_wtr.runThread();

    // If a current running system, ensure hb of current running trackers
    // are Registered prior to receiving any durable votes - votes are
    // checked to verify they are cast for known trackers.
    rti::util::sleep(dds::core::Duration(1));
    vote_rdr.runThread();

    // *** START WRITER LISTENERS or MONITOR THREADS (This step Optional)
    //servo_writer.runThread() # start a statuses monitor thread on the DA Writer
    //or...#listener, Heartbeat is periodic and will run as a thread
    DefaultWriterListener * listener = new DefaultWriterListener; 
    servo_writer.getMyDataWriter().listener (listener, dds::core::status::StatusMask::all());


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
      // INITIALIZE, POSTINIT, VOTE, WAIT_FOR_VOTES, VOTE_RESULTS, STEADY_STATE,
      // SHUT_DOWN, ERROR.
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
      switch (redundancy_db.smState()) {	
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
	// after a new tracker hb is detected (still waiting for more trackers
	// if less than 3. This ensures that all trackers leave INITIALIZE
	// within ~100ms of eachother. Important, since any tracker still in
	// INITIALIZE or POSTINIT when a vote comes in is considered a late
	// joiner.
	std::cout << redundancy_db.numberOfTrackers() << ":" << std::flush;
	
	if (redundancy_db.numberOfTrackers()==redundancy_db.votesExpected() \
	      || redundancy_db.tenSecCount()) {
	    redundancy_db.setSM_State(POSTINIT);
	};
	break;

      case POSTINIT:
	// Pauses beifly to ensure a late joiner has process durable votes first
	// in the case heartbeats came more quickly and moved us out of INITIALIZE
	std::cout << "\nSTATE: POSTINIT" << std::endl;
	//std::cout << redundancy_db.getMyGuid() << std::endl;
	
	rti::util::sleep(dds::core::Duration(1));

        redundancy_db.setSM_State(VOTE);	  
	break;

      case VOTE:
	// this state tranitions quickly once we vote and ensure one vote
	std::cout << "\nSTATE: VOTING" << std::endl;
	
	// For first time system up, all the trackers will see the other
	// two trackers heartbeats or 10sec expires after a new HB.
	// They should be in VOTE state within 100ms of eachother. This
	// skew (should they vote with out delay) could allow a tracker to see
	// itself as a late joiner.
	rti::util::sleep(dds::core::Duration(1));

	vote_wtr.vote(); 
	redundancy_db.setSM_State(WAIT_VOTES_IN);
	  
	cycle_cnt=5; // make sure we print the first entry to each state
	break;

      case WAIT_VOTES_IN:
	if (!(cycle_cnt++ %5)) {
	  cycle_cnt = 1;
	  std::cout << "\nSTATE: WAITING FOR ALL VOTES" << std::endl;
	};
	// wait for all votes to be in, if < 3 the timing is dependent
	// upon delays from different trackers starting.
        if (redundancy_db.votesIn() == redundancy_db.numberOfTrackers()) 
	  redundancy_db.setSM_State(VOTE_RESULTS);
	break;

      case VOTE_RESULTS: 
	std::cout << "\nSTATE: ASSESSING VOTING RESULTS" << std::endl;
	// std::cout << redundancy_db.getMyGuid() << std::endl;
	
	// Assess if all trackers are consistent - fault any missing
	// or inconsistent trackers. Set Pixy_Servo_Strength based on
	// results: PRIMARY 30, SECONDARY 20, TERTIARY 10
	redundancy_db.assessVoteResults();
	// redundancy_db.printSortedTrackers();

	// change my ownership strength based on my roll.
        ownership_strength_value = redundancy_db.getMyRollStrength();
	ownership_strength.value(ownership_strength_value);
	writer_qos << ownership_strength;
	servo_writer.getMyDataWriter().qos(writer_qos);
	servo_writer.enable(); // enable after we've set the strength

	redundancy_db.setSM_State(STEADY_STATE);
	cycle_cnt=0; // next cycle triggers a print upon entry to STEADY_STATE
	break;
	
      case STEADY_STATE:
	// print the first time we enter state
	if (cycle_cnt==0 || !(cycle_cnt%TEN_SEC)) {
	  std::cout << "STATE: STEADY_STATE" << std::endl;
	  // std::cout << redundancy_db.getMyGuid() << std::endl;
	  redundancy_db.printMyState();
	}

	servo_writer.printGimbalPosition();	  
	// Check for lost tracker every one sec: HB deadline missed from a
	// tracker is 100ms (within 100ms the secondary's samples will be used
	// by DDS ownership deadline miss.) This logic is to consider formal
	// revote for a new primary - i.e. promoting the secondary. Votine time
	// is independent of the Secondary's samples being used and can be
	// done is a less urgent manner. 
	//   
	if (!(cycle_cnt++ %ONE_SEC)) // assess revote every one sec
	  for (int i=0; i<redundancy_db.numberOfTrackers(); i++){
	    if ((i !=redundancy_db.getMyOrdinal()-1) && \
		(redundancy_db.getTrackerState_ptr(i)->hbDeadlineCnt == 0)) {
	      // we need to drop this tracker and promote all lower trackers
	      redundancy_db.lostTracker(i);
	      redundancy_db.setSM_State(VOTE);
	      break;
	    } else {
	      // zero the count
	      redundancy_db.getTrackerState_ptr(i)->hbDeadlineCnt = 0;
	    }
	  } // for

	// Discovered a late joining Tracker, bring in silently at next
	// available roll {Secondary, Tertiary}
	if (redundancy_db.isNewTracker()) {
	  ;
	}
	// while the background update LEDs and check our ordinal
	// An invalid ordinal indicates a software bug.
	if (!redundancy_db.validateMyOrdinal()) {
	  std::cerr << "Ordinal Failure: "
		    << redundancy_db.getMyOrdinal() 
		    << " " << redundancy_db.getMyGuid()
		    << std::endl;
	  redundancy_db.setSM_State(ERROR);
	}
	
	break;
	
      case SHUT_DOWN:
	std::cout << "\nSTATE: SHUT DOWN" << std::endl;	
	application::shutdown_requested = true;
	break;
	
      case ERROR: // detectable error state
	std::cout << "\nSTATE: ERROR" << std::endl;	
	// print message/red light LEDs and SHUT DOWN
	redundancy_db.setSM_State(SHUT_DOWN);
	break;
	
      default:
	;
      } // switch
      rti::util::sleep(MAINLOOP_PERIOD);
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

