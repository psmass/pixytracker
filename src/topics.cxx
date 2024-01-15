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

#include <dds/dds.hpp>
#include "topics.hpp"
#include "gimbal.hpp"
#include <rti/util/util.hpp> // for sleep

namespace MODULE
{

  // Array to Index map enum Roll
  enum Roll roll_array[4]{PRIMARY, SECONDARY, TERTIARY, UNASSIGNED};
  std::string roll_string_map[4] {"Primary", "Secondary", "Tertiary", "Not Voted Yet"};
  std::string state_string_map[8]{"INITIALIZE", "POSTINIT", "VOTE", "WAIT_VOTES_IN",\
      "VOTE_RESULTS", "STEADY_STATE", "SHUT_DOWN", "ERROR"};
  
  /* Helper functions to covert between InstanceHandles, GUIDs and arrays
     Instances is what I can access, Guids for math, arrays to place as
     sequences in written samples.
   */
  // need to covert Instance Handles to Guids to use the math operators to compare values
  rti::core::Guid convertToGuid ( const dds::core::InstanceHandle& instanceHandle ) {
    rti::core::Guid guid;
    memcpy(&guid,reinterpret_cast<DDS_Octet const *>(&instanceHandle ),16);
    return guid;
  }

  // routing to convert an int array to guid (extact sample sequence to guid)
  rti::core::Guid convertIArrayToGuid ( const uint8_t *array )
  {
    rti::core::Guid guid;
    memcpy(&guid,array,16);
    return guid;
  }

  // routine to put Guid in an array of 8 bit ints (to place in a sample sequence)
  void convertGuidToIArray (uint8_t *array, rti::core::Guid guid )
  {
    memcpy(array,reinterpret_cast<DDS_Octet const *>(&guid),16);
  }

  
  RedundancyDb::RedundancyDb(const dds::domain::DomainParticipant participant)
  {
    // initialize guid tracking array to my_guid followed by 0's
    uint8_t iarr[16] {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    rti::core::Guid ff_guid= convertIArrayToGuid(iarr);
    this->ff_guid = ff_guid; // save this guid
    
    this->array_tracker_states[0].operational_hb=OK; // set myself operational
    //     this->array_tracker_states[0].guid (my guid) set in HB writer c'tor
    this->array_tracker_states[1].guid=ff_guid;
    this->array_tracker_states[2].guid=ff_guid;

    // initialize the ordered array of tracker ptrs
    for (int i=0; i<3; i++)
      ordered_array_tracker_state_ptrs[i]=&array_tracker_states[i];
  }

  
  void RedundancyDb::printSortedTrackers() {
      std::cout << "DISCOVERED SORTED TRACKERS " << std::endl;
      std::cout << "This Tracker is Ordinal: " << this->my_ordinal
		<< " Trackers cnt: " << this->number_of_trackers
		<< std::endl;
      for (int i=0; i<3; i++) 
	std::cout << this->ordered_array_tracker_state_ptrs[i]->guid
		  << " - "
		  << roll_string_map[this->ordered_array_tracker_state_ptrs[i]->roll]
		  << " hb_cnt:"
		  << this->ordered_array_tracker_state_ptrs[i]->hbDeadlineCnt
		  << std::endl;
    }

  
  int RedundancyDb::getMyRollStrength()
  {
    int ownership_strength_roll_map[3] {30,20,10};
    return ownership_strength_roll_map					\
      [this->ordered_array_tracker_state_ptrs[this->my_ordinal-1]->roll];
  }

  void RedundancyDb::sortSaveGuids()
  {
    TrackerState* temp_tracker_state_ptr;
    for (int l=0; l<2; l++) {
	
      for (int i=0; i<2; i++) {  // we are only bubble up at most 2
	if (this->ordered_array_tracker_state_ptrs[i]->guid >
	    this->ordered_array_tracker_state_ptrs[i+1]->guid) {
	  // we are going to swap the pointers to order the pointer array
	  temp_tracker_state_ptr = this->ordered_array_tracker_state_ptrs[i];

	  // bubling up or down so adjust the ordinal - ordinals 1 based
	  if ( this->my_ordinal-1 == i) // bubbling  up
	    this->my_ordinal++; 
	  else if (this->my_ordinal-1 == i+1) // bubbling  down
	    this->my_ordinal--;  
	  if (this->my_ordinal < 1 || this->my_ordinal > 3) 
	    // This should never occur but can cause a null ptr exception
	    std::cerr << "ORDINAL FAILURE = " << this->my_ordinal << std::endl;
		
	  this->ordered_array_tracker_state_ptrs[i] =		\
	    this->ordered_array_tracker_state_ptrs[i+1];
	  this->ordered_array_tracker_state_ptrs[i+1] = temp_tracker_state_ptr;
	}
      } // for i
    } // for l
      
    this->printSortedTrackers();
  }

  
  void RedundancyDb::lostTracker(int tracker_indx)
  {
    // Lost a tracker(tracker_ordinal = tracker_indx+1).
    // We need to promote all lower trackers
    // first clear the trackerStat struct[i]. The trackers in order so we just
    // need to shuffle them forward in the ordered_array_tracker_state_ptrs, and
    // promote thier roll.
    // Note: There must be at least 2 trackers as we are 1 and we lost 1.

    // save lost tracker roll
    enum Roll lost_tracker_roll = \
      this->ordered_array_tracker_state_ptrs[tracker_indx]->roll;
    std::cout << "Lost Tracker Roll is: " << roll_string_map[lost_tracker_roll] << std::endl;
    
    // zero out the lost tracker, drop the number of trackers and resort
    this->ordered_array_tracker_state_ptrs[tracker_indx]->guid =\
      this->ff_guid;
    this->ordered_array_tracker_state_ptrs[tracker_indx]->roll = \
      UNASSIGNED;
    this->clearVotesTracker(tracker_indx); // clears Ivoted as well
    this->ordered_array_tracker_state_ptrs[tracker_indx]->inconsistent_vote=FAILED;
    this->number_of_trackers--; 
    this->sortSaveGuids(); // force reordering of the array
    this->is_new_tracker = false; // vote null Guids for lost tracker

    // promote all lower non null guid trackers
    for (int i=0; i<3; i++) {
      // enum Primary 0, Secondary 1, Tertiary 3
      if (this->ordered_array_tracker_state_ptrs[i]->roll > lost_tracker_roll &&
	  this->ordered_array_tracker_state_ptrs[i]->roll !=UNASSIGNED) {
	enum Roll t_roll = this->ordered_array_tracker_state_ptrs[i]->roll;
        this->ordered_array_tracker_state_ptrs[i]->roll = roll_array[t_roll-1]; 
	  // roll_array[(this->ordered_array_tracker_state_ptrs[i]->roll)-1];
	std::cout << "LostTracker() promoted: from: "
		  << roll_string_map[t_roll]  << " to: "
		  << roll_string_map[this->ordered_array_tracker_state_ptrs[i]->roll]
		  << " temp cmpr: "
		  << roll_string_map[t_roll-1]
		  << std::endl;
      }
     }
    this->printSortedTrackers();

  }
    
  
  void RedundancyDb::assessVoteResults(void)
  {
    /*
    std::cout << "My Tracker Guid: "
	      << this->ordered_array_tracker_state_ptrs[this->my_ordinal-1]->guid
	      << " My Ordinal: " << this->my_ordinal
	      << std::endl;
    */
    
    // Each tracker should show votes for one roll = number of trackers and
    // the rest 0. NOTE: while initially tracker order and rolls are correlated
    // (i.e. Primary is ordinal 1, Secondary ordinal 2, Tertiary 3, over time
    // this can change.
    // Go through the ordered array of trackers and validate winner for
    // their roll and consistency. If inconsistent index of non-zero looser
    // index is the faulty tracker.

    int roll_vote {0}, largest_roll_vote_idx {0}, largest_roll_vote_tally {0};
    
    for (int ord=0; ord<this->number_of_trackers; ord++) {
      // check one vote = number_of_trackers and all others are 0
      // For a single fault in a tripple redundant system, one roll_vote
      // might be 2 while one of the others is 1. Vote of 1,1,1 would
      // be a double fault. Possible consistent states are:
      //      1. (3,0,0) (0,3,0), (0,0,3),
      //      2. Faulted or missing unit - (2,0,0), (0,2,0), (0,0,2), 
      //      3. Simplex (1,0,0), (0,1,0), (0,0,1).
      //
      // Inconsistent vote state would be:
      //      1. Any vote that does not add up to the number of trackers.
      //         But this cannot occur, since when we read the vote sample, we
      //         pulled number_of_trackers of guids out of it. DDS would have
      //         asserted if we had read a null field).
      //      2. (2,1,0), (2,0,1), (1,1,1),        
      //          F[1]     F[3]     double fault - everyone voted different
      //      3. (0,1,1), (1,0,1), (1,1,0)        (and amigious -who's right?)
      //          Double faults and ambigious.
      //
      //       We won't check for double faults - TODO: Assert() if a double fault
      //
      largest_roll_vote_tally = 0; 
      for (int roll_idx=0; roll_idx<this->number_of_trackers; roll_idx++) {
	roll_vote=this->ordered_array_tracker_state_ptrs[ord]->votes[roll_idx];
	if (roll_vote > largest_roll_vote_tally) { // track winner
	    largest_roll_vote_tally = roll_vote;
	    largest_roll_vote_idx = roll_idx;
	}
	// This checks for ALL inconsistent votes and marks faulted tracker
	if(roll_vote <this->number_of_trackers) { // not unanomous
	  this->ordered_array_tracker_state_ptrs[ord]->inconsistent_vote=FAILED;
	}
      } // for roll_idx

      // assign the winning roll to this tracker
      this->ordered_array_tracker_state_ptrs[ord]->roll=roll_array[largest_roll_vote_idx];
    } // for ord
    this->printVoteResults();
  };

     
  ServoWtr::ServoWtr(
	const dds::domain::DomainParticipant participant,
        bool periodic,	
        dds::core::Duration period)
    : Writer(participant, "ServoControl", "publisher::servo_topic_writer", periodic, period)
  {
        // Update Static Topic Data parameters in the beginning of the handler
        // prior to the loop, but after the entity base class creates the sample.
        // std::cout << "Servo Writer C'Tor" << std::endl;
    this->getMyDataSample()->value<uint16_t>("frequency", SERVO_FREQUENCY_HZ);
  };

  void ServoWtr::writeData(int32_t x, int32_t y)
  {
    if (this->enabled_to_write) {
      gimbal.update_pan(x);
      gimbal.update_tilt(y);
      this->getMyDataSample()->value<uint16_t>("pan", gimbal.get_pan_position());
      this->getMyDataSample()->value<uint16_t>("tilt", gimbal.get_tilt_position());
      this->topicWriter.write(*this->getMyDataSample());
    }
  };

  
  ShapesRdr::ShapesRdr(const dds::domain::DomainParticipant participant,  ServoWtr* servoWriter)
    : Reader(participant, "ShapeTypeExtended", "subscriber::shape_topic_reader")
  {
    this->servo_writer = servoWriter;
  };

  void ShapesRdr::handler(dds::core::xtypes::DynamicData& data)
  {
    DDS_ReturnCode_t retcode;
    //std::cout << "Shapes Reader Handler Executing" << std::endl;
    // Control the pan & til
    x = data.value<int32_t>("x");
    y = data.value<int32_t>("y");
    //std::cout << "X: " << x <<  "   Y: " << y << "\r" << std::endl;
    this->servo_writer->writeData(x,y);

  };

  HeartbeatWtr::HeartbeatWtr(
			     const dds::domain::DomainParticipant participant,
			     RedundancyDb* redundancy_db_obj,
			     bool periodic,
			     dds::core::Duration period) 
    : Writer(participant, "TrackerHeartbeat", \
	     "publisher::tracker_hb_topic_writer",\
	     periodic, period)
  {
    this->redundancy_db_obj = redundancy_db_obj;

    /* Get my Heartbeat writer Instance Handle to use as my unique identifier.
       Note: we need the HB Writer (instance) GUID to compare should HB missed
       Deadline trigger to know which instance to null out and revote.
       We'll need to convert instance handles to GUIDS to use the math operators. 
       Note sure how to get Guid directly and can't do math on Instance handle.
     */
    const dds::core::InstanceHandle handle=this->getMyDataWriter()->instance_handle();
    //std::cout << "INSTANCE HANDLE: " << handle << std::endl;

    // initially all trackers place their own guid in the [0] index prior to
    // bubble sorting the ordered_array_tracker_state_ptrs[3]
    rti::core::Guid guid = convertToGuid(handle);
    this->redundancy_db_obj->getMyTrackerStatePtr()->guid = guid;
    this->redundancy_db_obj->setMyGuid(guid);
    std::cout << "MY GUID: " << guid << std::endl;
    
    // get my guid and place it in the heartbeat sample (note it's actually already there
    // in the meta data, but we need to key on it so as to detect the HB Wtr instance
    // should it miss a deadline (i.e. fail)
    uint8_t iarr[16];
    convertGuidToIArray(iarr, guid);

    std::vector<uint8_t> seq_values;
    for (int i=0; i<16; i++)
      seq_values.push_back(iarr[i]);
    
    this->getMyDataSample()->set_values("MyHBwriterHandle", seq_values);
  }
  
    
  // write() is effectively a runtime down cast for periodic data
  void HeartbeatWtr::write(void)
  {
    this->topicWriter.write(*this->getMyDataSample());
  };
  

  HeartbeatRdr::HeartbeatRdr(const dds::domain::DomainParticipant participant,
			     RedundancyDb* redundancy_db_obj)
    : Reader(participant, "TrackerHeartbeat", "subscriber::tracker_hb_topic_reader")
  {
    this->redundancy_db_obj = redundancy_db_obj;
  };

  void HeartbeatRdr::handler(dds::core::xtypes::DynamicData& data)
  {
    // std::cout << "Received Heartbeat: GUID=";

    std::vector<uint8_t> seq_values = data.get_values<uint8_t>("MyHBwriterHandle");
    
    uint8_t iarr[16];
    for (int i=15; i>=0; i--) {
      iarr[i]=seq_values.back();
      seq_values.pop_back();
    } 
    rti::core::Guid hb_guid = convertIArrayToGuid(iarr);
    
    bool duplicate {false};
    // if an existing hb guid, increment my hb dead man/deadline count of
    // the corresponding tracker
    for (int i=0; i<this->redundancy_db_obj->numberOfTrackers(); i++)
      if (hb_guid==this->redundancy_db_obj->getTrackerState_ptr(i)->guid) {
	 duplicate=true;
	 this->redundancy_db_obj->getTrackerState_ptr(i)->hbDeadlineCnt++;
	}

    // if new tracker we have available tracker space, find available slot in
    // tracker array populate it and sort/resort.
    if  (!duplicate && (this->redundancy_db_obj->numberOfTrackers() < 3)) {
      this->redundancy_db_obj->incNumberOfTrackers();
      this->redundancy_db_obj->resetTenSecCount(); 
      for (int i=0; i<3; i++)
	if (this->redundancy_db_obj->getTrackerState_ptr(i)->guid == \
	    this->redundancy_db_obj->getNullGuid()) {
	  this->redundancy_db_obj->getTrackerState_ptr(i)->guid = hb_guid;
	  this->redundancy_db_obj->getTrackerState_ptr(i)->operational_hb = OK;
	  this->redundancy_db_obj->getTrackerState_ptr(i)->hbDeadlineCnt = 1;
	  this->redundancy_db_obj->sortSaveGuids();
	  this->redundancy_db_obj->setNewTracker(true);
	  break;
	} 
      // std::cout << hb_guid << std::endl;
    }	   
  };


  VoteWtr::VoteWtr(const dds::domain::DomainParticipant participant,
		   RedundancyDb* redundancy_db_obj,
		   bool periodic,
		   dds::core::Duration period)
    : Writer(participant, "VoteType", \
	     "publisher::vote_topic_writer",\
	     periodic, period)
  {
     this->redundancy_db_obj = redundancy_db_obj;
     this->setSampleField("SourceHBwriterHandle", redundancy_db_obj->getMyGuid());
     // initialize all vote samples to NULL guid (so we don't have to worry
     // about accessing an N/A field
     for (int i=0; i<3; i++) {
	this->setSampleField(roll_string_map[i], \
			     redundancy_db_obj->getNullGuid());
      }

  };

  
  void  VoteWtr::setSampleField (std::string topic_field, rti::core::Guid guid)
  {
    // Helper function to covert topic int array[16] Guids to rti::core:Guid
    uint8_t iarr[16];
    convertGuidToIArray(iarr, guid);

    std::vector<uint8_t> seq_values;
    for (int i=0; i<16; i++)
      seq_values.push_back(iarr[i]);

    this->getMyDataSample()->set_values(topic_field, seq_values);
  };  


  void VoteWtr::vote()
  {
    // Initialize the sample (source guid filled out in c'tor)
    // set Initialize all votes to null_guid
    for (int i=0; i<3; i++)
      this->setSampleField(roll_string_map[roll_array[i]],		\
			   redundancy_db_obj->getNullGuid());      

    // Set the number of trackers we are voting for (non-null guids)
    this->getMyDataSample()->value("NumberOfTrackers", \
				    redundancy_db_obj->numberOfTrackers());

			       
    // set the votes according to if the system was operational or not.
    //
    if (!redundancy_db_obj->isLateJoiner()) //was not operational,
      // no rolls assigned. Assign them in order of sorted guids
      // lowest {Primary} to higher
      for (int i=0; i<redundancy_db_obj->numberOfTrackers(); i++) {
	redundancy_db_obj->getTrackerState_ptr(i)->roll=roll_array[i];
      }
    // else was operational and the durable votes have fill the db
    // At this point the ordered db has all rolls assigned
 
    for (int i=0; i<redundancy_db_obj->numberOfTrackers(); i++)
	  this->setSampleField( \
		      roll_string_map[redundancy_db_obj->getTrackerState_ptr(i)->roll], \
		      redundancy_db_obj->getTrackerState_ptr(i)->guid
				);

    // We've populated the Vote Topic with our Selections according to the voting algorithm.
    // Now Read our vote topic back and register our own vote (since we won't receive it)
    for (int l=0; l<redundancy_db_obj->numberOfTrackers(); l++) {

      std::vector<uint8_t> seq_values_p = getMyDataSample()->get_values<uint8_t>(roll_string_map[l]);

      uint8_t iarr[16];
      for (int i=15; i>=0; i--) {
	iarr[i]=seq_values_p.back();
	seq_values_p.pop_back();
      }

      rti::core::Guid guid= convertIArrayToGuid(iarr);
      // update my vote array based on the roll (i)
      for (int i=0; i<redundancy_db_obj->numberOfTrackers(); i++)
	if (guid == redundancy_db_obj->getTrackerState_ptr(i)->guid)
	  redundancy_db_obj->getTrackerState_ptr(i)->
	    votes[redundancy_db_obj->getTrackerState_ptr(i)->roll]++;
	
    } // for
    redundancy_db_obj->getMyTrackerStatePtr()->Ivoted=true;    
  
    // cast our vote - write the vote sample
    this->topicWriter.write(*this->getMyDataSample());
  }

    
  VoteRdr::VoteRdr(const dds::domain::DomainParticipant participant,
		   RedundancyDb* redundancy_db_obj)
    : Reader(participant, "VoteType", "subscriber::vote_topic_reader")
  {
    this->redundancy_db_obj = redundancy_db_obj;    
  };
  

  rti::core::Guid VoteRdr::extractGuid (dds::core::xtypes::DynamicData& sample,
					std::string topicField)
  {
      std::vector<uint8_t> seq_values_p = sample.get_values<uint8_t>(topicField);
    
      uint8_t iarr[16];
      for (int i=15; i>=0; i--) {
	iarr[i]=seq_values_p.back();
	seq_values_p.pop_back();
      }

      rti::core::Guid guid= convertIArrayToGuid(iarr);
      return (guid);
  }


  void VoteRdr::handler(dds::core::xtypes::DynamicData& data)
  {
    // Votes can be received under three conditions (states):
    //
    //   INITIALIZE / POSTINIT (This tracker is a Late Joiner) - indicates
    //   the system  has been running, other trackers (1 or 2 of them) have
    //   voted and are in VOTE_RESULTS or more likely STEADY_STATE.
    //
    //   STEADY_STATE: (Late Joiner entered the system, and MUST vote the
    //   incumbants Primary and Secondary (if two trackers running) and add
    //   itself as Secondary or Tertiary ( 1 or 2 running trackers as
    //   indicated in the durable vote) the late joiner received during
    //   the INITIALIZE phase.
    //
    //   WAITING_VOTES_IN: This is the "normal" case when Trackers come
    //   on line together. Heartbeat timing resets, ensure that as
    //   Trackers come up within a 10sec window they all go though the
    //   POSTINIT and get to VOTE state within 100ms of eachother (so
    //   that none of the trackers coming up together see the others or
    //   themselves as a late joiner.
    //    
    std::cout << "\nReceived Vote during State "
	      << state_string_map[redundancy_db_obj->smState()];

    bool tracker_voted {true}; // set true so vote ignored by default
    bool known_tracker {false};
    
    int number_voted_trackers;

    number_voted_trackers  = (int)data.value<int32_t>("NumberOfTrackers");
    std::cout << " Number of trackers in vote: "
	      << number_voted_trackers;

    rti::core::Guid this_guid, guid[3];
    for (int i=0; i<3; i++) // initialize guid array to null Guid
      guid[i]=redundancy_db_obj->getNullGuid(); 

    // Verify source and integrity of the vote.
    // Verify Source: The vote must be from a known tracker, (ignore
    // unknown tracker  votes) this valid tracker has not already voted
    // (not supposed to), or dupplicate votes (i.e. if a the same
    // known tracker voted twice (technically an error).
    //
    this_guid = this->extractGuid(data, "SourceHBwriterHandle");
    std::cout << " From: " << this_guid << std::endl;
    for (int i=0; i<3; i++) { // check to 3 (possible null Guid compare
      if (this_guid == \
	  redundancy_db_obj->getTrackerState_ptr(i)->guid) {
	known_tracker = true;
	if (!redundancy_db_obj->getTrackerState_ptr(i)->Ivoted) {
	  tracker_voted = false; // tracker had not previously voted
	  redundancy_db_obj->getTrackerState_ptr(i)->Ivoted = true;
	} // else we'll ignore this vote (see if (!tracker_voted) below)
      }
    }
    if (!known_tracker) {
      std::cerr << "Received a Vote from unknown tracker" << std::endl;
      goto bad_vote;
    }
    
    std::cout << "Verified Source is consistent" << std::endl;
  
    // Verify integrity of the vote: A valid tracks votes are all unique
    // - i.e. it did not vote for the same tracker for two different
    // rolls, or vote for a null Guid. All votes are for known trackers.
    //
    if (!tracker_voted) { // complete validation of the vote
      for (int i=0; i<number_voted_trackers; i++) {
	// verify the integrity of the vote: no dupplicate votes
	// or votes for nullGuid
	guid[i] = this->extractGuid(data, roll_string_map[i]);
	std::cout << "Extracted from vote: " << guid[i]
		  << " For roll: " << roll_string_map[i]
		  << std::endl;
	// compare guid[i] to all prior voted guids for valid, non-dup
	for (int k=0; k<i; k++) 
	  if (guid[i]==guid[k] || \
	      guid[i]==redundancy_db_obj->getNullGuid())
	    goto bad_vote;
      }
    }
    std::cout << "Verified no votes for  dups or null Guids" << std::endl;
      
    // verify all guids voted for were for known trackers
    for (int i=0; i<number_voted_trackers; i++) {
      known_tracker = false;
      for (int k=0; k<3; k++) 
	if (guid[i] ==  redundancy_db_obj->getTrackerState_ptr(k)->guid)
	  known_tracker = true;
      if (!known_tracker)
	goto bad_vote;  
    }

    std::cout << "Verified all votes for known trackers" << std::endl;
    std::cout << "Vericiation complete " << std::endl;
      
    // At this point the vote is validated. Process it.
    switch (redundancy_db_obj->smState()) {

    case INITIALIZE:
    case POSTINIT:	
      // for each roll see which tracker is currently assigned and give
      // it the number of trackers plus 1 (our vote)
      
      for (int roll_idx=0; roll_idx<number_voted_trackers; roll_idx++) { 
	// see who's Guid we are getting a vote for?
	for (int i=0; i<3; i++) { // go through all tracker ordinals
	  if (guid[roll_idx] ==						\
	      redundancy_db_obj->getTrackerState_ptr(i)->guid) {
	    redundancy_db_obj->getTrackerState_ptr(i)->votes[roll_idx] \
	      = number_voted_trackers; // We'll add our vote when we vote()
	    redundancy_db_obj->getTrackerState_ptr(i)->roll 
	      = roll_array[roll_idx]; // set incumbant roll
	  }
	}
      } // for roll_idx;
	
      std::cout << "System Has been running - use old votes" << std::endl;
      // Set our own vote. Note number_of_voted trackers (1 or 2) tells us
      // if we are voting ourselves Secondary or Tertiary.
      // Roll enums are 0 based, so number_of trackers are 1, 2, 3.
      redundancy_db_obj->getMyTrackerStatePtr()->roll= \
	roll_array[number_voted_trackers];
      // note: we add our own vote to ourselves when we vote()
      redundancy_db_obj-> getMyTrackerStatePtr()-> \
	votes[number_voted_trackers] = number_voted_trackers;
      // no need to wait for next vote or 10 sec if the system had been
      // operational as we are only adding this tracker back.
      redundancy_db_obj->setLateJoiner(true);	
      std::cout << "Vote Reader Late Joining registered vote state" << std::endl;
      redundancy_db_obj->printVoteResults();
      break;

    case VOTE:
    case WAIT_VOTES_IN:
      std::cout << "System just up process new vote - " << std::endl;
      // go through and extact the vote Primary to Tertiary
      for (int roll_idx=0; roll_idx<number_voted_trackers; roll_idx++) {
	// see who's Guid we are getting a vote for?
	for (int i=0; i<number_voted_trackers; i++) {
	  if (guid[roll_idx] ==						\
	      redundancy_db_obj->getTrackerState_ptr(i)->guid)
	    // increment the vote for tracker based on the roll we are checking
	    redundancy_db_obj->getTrackerState_ptr(i)->votes[roll_idx]++;
	}
      }  // for roll_idx;
      break;
      
    case VOTE_RESULTS:
    case STEADY_STATE:
      // A vote received after voting is a late joining Tracker. All were
      // interested in is to update our tracker db with the late joins roll.      
      for (int roll_idx=0; roll_idx<number_voted_trackers; roll_idx++)
	if (this_guid == guid[roll_idx])
	  redundancy_db_obj->getTrackerState_ptr(roll_idx)->roll \
	    = roll_array[roll_idx];
      break;
    
    case SHUT_DOWN:
    case ERROR:
    default:// do nothing for these states
      break;
      
    } // if tracker did not vote

  done:
    redundancy_db_obj->incVotesIn(); //  inc total vote tally
    return;
    
  bad_vote:
    std::cerr << "Vote found inconsistent from source: "
	      << this_guid
	      << std::endl;
    //TODO: Mark that source as failed and ignore heartbeats  
  };
    
} // namespace MODULE

