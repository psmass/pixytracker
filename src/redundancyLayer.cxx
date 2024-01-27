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
#include <rti/util/util.hpp> // for sleep
#include "redundancyLayer.hpp"

namespace MODULE
{

  // Array to Index map enum Role
  enum Role role_array[4]{PRIMARY, SECONDARY, TERTIARY, UNASSIGNED};
  std::string role_string_map[4] {"Primary", "Secondary", "Tertiary", "Not Voted Yet"};
  std::string state_string_map[8]{"INITIALIZE", "PREVOTE", "VOTE", "WAIT_VOTES_IN",\
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

  
  void RedundancyDb::printSortedTrackers()
  {
    for (int i=0; i<3; i++) 
      std::cout << this->ordered_array_tracker_state_ptrs[i]->guid
		<< " - "
		<< role_string_map[this->ordered_array_tracker_state_ptrs[i]->role]
		<< std::endl;
    }

  
  int RedundancyDb::getMyRoleStrength()
  {
    int ownership_strength_role_map[3] {30,20,10};
    return ownership_strength_role_map					\
      [this->ordered_array_tracker_state_ptrs[this->my_ordinal-1]->role];
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
      
    // this->printSortedTrackers();
  }

  
  void RedundancyDb::lostTracker(int tracker_indx)
  {
    // Lost a tracker(tracker_ordinal = tracker_indx+1).
    // We need to promote all lower trackers
    // first clear the trackerStat struct[i]. The trackers in order so we just
    // need to shuffle them forward in the ordered_array_tracker_state_ptrs, and
    // promote thier role.
    // Note: There must be at least 2 trackers as we are 1 and we lost 1.

    // save lost tracker role
    enum Role lost_tracker_role = \
      this->ordered_array_tracker_state_ptrs[tracker_indx]->role;

    // zero out the lost tracker, drop the number of trackers and resort
    this->ordered_array_tracker_state_ptrs[tracker_indx]->guid =\
      this->ff_guid;
    this->ordered_array_tracker_state_ptrs[tracker_indx]->role = \
      UNASSIGNED;
    this->ordered_array_tracker_state_ptrs[tracker_indx]->inconsistent_vote=FAILED;
    this->ordered_array_tracker_state_ptrs[tracker_indx]->operational_hb=FAILED;
    this->number_of_trackers--; 
    this->sortSaveGuids(); // force reordering of the array
    this->is_new_tracker = false; // vote null Guids for lost tracker

    // promote all lower non null guid trackers
    for (int i=0; i<3; i++) {
      // enum Primary 0, Secondary 1, Tertiary 3
      if (this->ordered_array_tracker_state_ptrs[i]->role > lost_tracker_role &&
	  this->ordered_array_tracker_state_ptrs[i]->role !=UNASSIGNED) {
	enum Role t_role = this->ordered_array_tracker_state_ptrs[i]->role;
        this->ordered_array_tracker_state_ptrs[i]->role = role_array[t_role-1]; 
      }
     }
    // this->printSortedTrackers();

  }
  
  void RedundancyDb::printBallotVoteTracker(int t) {
	std::cout << "\nFor Tracker: " \
		  << this->ordered_array_tracker_state_ptrs[t]->guid \
		  << std::endl;
	for (int i=0; i<3; i++)
	  std::cout<< "Role: " \
		   << role_string_map[i] \
		   << " " \
		   << this->ordered_array_tracker_state_ptrs[t]->votes[i] \
		   << std::endl;
  }

  
  void RedundancyDb::printFullBallot(void)
  {
      for (int i=0; i<this->numberOfTrackers(); i++)
	this->printBallotVoteTracker(i);
   }

   
  bool RedundancyDb::validateBallot(void)
  {
    // private function to ensure this trackers ballot (potential vote) is valid
    // should be called once the ballot is ready for vote since it checks
    // for unanomous vote, nothing more or less
    int vote_tally[3] {0, 0, 0};
    for (int i=0; i<this->number_of_trackers; i++) {
      for (int role_idx=0; role_idx<3; role_idx++) {
	// if a 0 or number_of_trackers load the vote_tally[role_idx]
	if (this->ordered_array_tracker_state_ptrs[i]->votes[role_idx] == this->number_of_trackers \
	    or this->ordered_array_tracker_state_ptrs[i]->votes[role_idx] == 0 ) {
	  vote_tally[role_idx] =this->ordered_array_tracker_state_ptrs[i]->votes[role_idx];
	} else {
	  std::cerr << "ERROR: Ballot Inconsistent " << std::endl;
	  //this->printBallotVoteTracker(i);
	}
      }
    }
    // we verified each vote for a role is either 0 or number_of_trackers
    // now ensure no role was voted twice
    if (vote_tally[0]+vote_tally[1]+vote_tally[2] == this->number_of_trackers)
      return true;
    else
      return false;
  }

  
  void RedundancyDb::assessVoteResults(void)
  {   
    // Each tracker should show votes for one role = number of trackers and
    // the rest 0. NOTE: while initially tracker order and roles are correlated
    // (i.e. Primary is ordinal 1, Secondary ordinal 2, Tertiary 3, over time
    // this can change.
    // Go through the ordered array of trackers and validate winner for
    // their role and consistency. If inconsistent index of non-zero looser
    // index is the faulty tracker.

    int role_vote {0}, largest_role_vote_idx {0}, largest_role_vote_tally {0};
    
    for (int ord=0; ord<this->number_of_trackers; ord++) {
      // check one vote = number_of_trackers and all others are 0
      // For a single fault in a tripple redundant system, one role_vote
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
      largest_role_vote_tally = 0; 
      for (int role_idx=0; role_idx<this->number_of_trackers; role_idx++) {
	role_vote=this->ordered_array_tracker_state_ptrs[ord]->votes[role_idx];
	if (role_vote > largest_role_vote_tally) { // track winner
	    largest_role_vote_tally = role_vote;
	    largest_role_vote_idx = role_idx;
	}
	// This checks for inconsistent / non unanomous vote, mark faulted tracker
	if(role_vote <this->number_of_trackers) { // not unanomous
	  this->ordered_array_tracker_state_ptrs[ord]->inconsistent_vote=FAILED;
	} else
	  this->ordered_array_tracker_state_ptrs[ord]->inconsistent_vote=OK;	  
      } // for role_idx

      // assign the winning role to this tracker
      this->ordered_array_tracker_state_ptrs[ord]->role=role_array[largest_role_vote_idx];
    } // for ord
    // this->printFullBallot();
    this->clearVotes(); // clear for next vote
  };

  void RedundancyDb::printMyState(void) {
    // This is the equivalent of what the 4 LEDs on a Rpi will provide.
    // Forth LED (l to r) 
    //
    // A Green LED in the associated Role indicates this Trackers Role. 
    // A Red LED indicates an associated Failed or missing Tracker of
    // tat role. OFF indicates the associated tracker of that roll is OK.
    //
    // The STATUS indicates this tracker status - Green OK, RED Failed.
    // This LED is useful as the track comes into operation, prir to role
    // assignement or if the trackers software detects a failure.
    //
    std::cout << "|   PRIMARY  | SECONDARY  |  TERTIARY  |  STATUS |" << std::endl;
    std::cout << "+------------+------------+------------+---------+" << std::endl;
    std::cout << "|";

    int i;
    for (int role=0; role<3; role++) { // primary secondary teriary 
      for (i=0; i<3; i++) {
	if (this->ordered_array_tracker_state_ptrs[i]->role
	    == role_array[role] ) {
	  if (this->ordered_array_tracker_state_ptrs[i]->guid
	      == this->my_guid )
	    std::cout << "  OK (Grn)  |"; // if me
	  else  // printStatus
	    std::cout << "            |";
	  
	  break;
	  }
      } // for i
      if (i==3) // no role assigned and not me
	    std::cout << "    RED     |";  
    } // for role 
    std::cout << "  GREEN  |" << std::endl;  // set my Status Green
    std::cout << "+------------+------------+------------+---------+" << std::endl;
  }

  void RedundancyDb::updateLedStatus (LedControl* led_control) {
    int i;
    for (int role=0; role<3; role++) { // primary secondary teriary
      for (i=0; i<3; i++) {
	if (this->ordered_array_tracker_state_ptrs[i]->role
	    == role_array[role] ) {
	  if (this->ordered_array_tracker_state_ptrs[i]->guid
	      == this->my_guid )
	    led_control->setLedGreen(role);
	  else // else not me so turn led off
	    led_control->setLedOff(role);

	  break; // processed role
	}
      }
      if (i==3) // no role assigned and not me
	led_control->setLedRed(role);
    } // for role
    led_control->setLedGreen(3); // set My Status Green (I'm running)
  }

  
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

    // initially all trackers place their own guid in the [0] index prior to
    // bubble sorting the ordered_array_tracker_state_ptrs[3]
    rti::core::Guid guid = convertToGuid(handle);
    this->redundancy_db_obj->getMyTrackerStatePtr()->guid = guid;
    this->redundancy_db_obj->setMyGuid(guid);
    
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
	this->setSampleField(role_string_map[i], \
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
      this->setSampleField(role_string_map[role_array[i]],		\
			   redundancy_db_obj->getNullGuid());      

    // Set the number of trackers we are voting for (non-null guids)
    this->getMyDataSample()->value("NumberOfTrackers", \
				    redundancy_db_obj->numberOfTrackers());

    // set the votes according to if the system this tracker was operational 
    // and not a Late Joiner
    if (!redundancy_db_obj->iWasOperational() \
	&& !redundancy_db_obj->isLateJoiner()) 
      // no roles assigned. Assign them in order of sorted guids
      // lowest {Primary} to higher
      for (int i=0; i<redundancy_db_obj->numberOfTrackers(); i++) 
	redundancy_db_obj->getTrackerState_ptr(i)->role=role_array[i];

    else  // is a late joiner - check for and fix an outlier case where
      // Trackers come up one at a time. The first tracker will only
      // vote for itself (no other trackers and does not vote again)
      // The second tracker will be a late joiner and votes fine.
      // The third tracker may have two Secondaries in its db since it
      // gets the first durable vote and make itself Secondary and then
      // gets the second durable vote which has the second tracker up
      // as secondary. In this case make sure this tracker is Tertiary
      if (redundancy_db_obj->numberOfTrackers()==3) {
	redundancy_db_obj->getMyTrackerStatePtr()->role=TERTIARY;
	// and reset our vote for ourselves as secondary to 0
	redundancy_db_obj->getMyTrackerStatePtr()->votes[1]=0;
      }

    redundancy_db_obj->setLateJoiner(false); // clear once voted
    // else was operational and the durable votes have fill the db
    // At this point the ordered db has all roles assigned
 
    for (int i=0; i<redundancy_db_obj->numberOfTrackers(); i++)
	  this->setSampleField( \
		      role_string_map[redundancy_db_obj->getTrackerState_ptr(i)->role], \
		      redundancy_db_obj->getTrackerState_ptr(i)->guid
				);

    // We've populated the Vote Topic with our Selections according to the voting algorithm.
    // Now Read our vote topic back and register our own vote (since we won't receive it)
    for (int l=0; l<redundancy_db_obj->numberOfTrackers(); l++) {

      std::vector<uint8_t> seq_values_p = getMyDataSample()->get_values<uint8_t>(role_string_map[l]);

      uint8_t iarr[16];
      for (int i=15; i>=0; i--) {
	iarr[i]=seq_values_p.back();
	seq_values_p.pop_back();
      }

      rti::core::Guid guid= convertIArrayToGuid(iarr);
      // update my vote array based on the role (i)
      for (int i=0; i<redundancy_db_obj->numberOfTrackers(); i++)
	if (guid == redundancy_db_obj->getTrackerState_ptr(i)->guid)
	  redundancy_db_obj->getTrackerState_ptr(i)->
	    votes[redundancy_db_obj->getTrackerState_ptr(i)->role]++;
	
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
    rti::core::Guid guid;
    
    if (topicField != " NotVoted Yet") {
      std::vector<uint8_t> seq_values_p = sample.get_values<uint8_t>(topicField);
    
      uint8_t iarr[16];
      for (int i=15; i>=0; i--) {
	iarr[i]=seq_values_p.back();
	seq_values_p.pop_back();
      }
     
      guid = convertIArrayToGuid(iarr);
    } else {
      std::cerr << "ERROR: Extract -  Invalid Topic Field: "
	        << topicField << std::endl;
      guid = redundancy_db_obj->getNullGuid();
    }	     
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

    bool tracker_voted {true}; // set true so vote ignored by default
    bool known_tracker {false};
    
    int number_voted_trackers;

    number_voted_trackers  = (int)data.value<int32_t>("NumberOfTrackers");

    rti::core::Guid source_guid, guid[3];
    for (int i=0; i<3; i++) // initialize guid array to null Guid
      guid[i]=redundancy_db_obj->getNullGuid(); 

    // Verify source and integrity of the vote.
    // Verify Source: The vote must be from a known tracker, (ignore
    // unknown tracker  votes) this valid tracker has not already voted
    // (not supposed to), or dupplicate votes (i.e. if a the same
    // known tracker voted twice (technically an error).
    //
    source_guid = this->extractGuid(data, "SourceHBwriterHandle");
    for (int i=0; i<3; i++) { // check to 3 (possible null Guid compare
      if (source_guid == \
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
  
    // Verify integrity of the vote: A valid tracks votes are all unique
    // - i.e. it did not vote for the same tracker for two different
    // roles, or vote for a null Guid. All votes are for known trackers.
    //
    if (!tracker_voted) { // complete validation of the vote
      for (int i=0; i<number_voted_trackers; i++) {
	// verify the integrity of the vote: no dupplicate votes
	// or votes for nullGuid
	guid[i] = this->extractGuid(data, role_string_map[i]);

	// compare guid[i] to all prior voted guids for valid, non-dup
	for (int k=0; k<i; k++) 
	  if (guid[i]==guid[k] || \
	      guid[i]==redundancy_db_obj->getNullGuid())
	    goto bad_vote;
      }
    }
      
    // verify all guids voted for were for known trackers
    for (int i=0; i<number_voted_trackers; i++) {
      known_tracker = false;
      for (int k=0; k<3; k++) 
	if (guid[i] ==  redundancy_db_obj->getTrackerState_ptr(k)->guid)
	  known_tracker = true;
      if (!known_tracker)
	goto bad_vote;  
    }
      
    // At this point the vote is validated. Process it.
    switch (redundancy_db_obj->smState()) {

    case INITIALIZE:
    case PREVOTE:
      // for each tracker populate thier role and  assign unanomous votes
      // to 
      
      for (int role_idx=0; role_idx<number_voted_trackers; role_idx++) { 
	// see who's Guid we are getting a vote for?
	for (int i=0; i<3; i++) { // go through all tracker ordinals
	  if (guid[role_idx] ==						\
	      redundancy_db_obj->getTrackerState_ptr(i)->guid) {
	    // update with unanomous vote ofr the incumbant role
	    redundancy_db_obj->getTrackerState_ptr(i)->votes[role_idx] \
	      = number_voted_trackers; // We'll add our vote when we vote()
	    // update their role in my db
	    redundancy_db_obj->getTrackerState_ptr(i)->role 
	      = role_array[role_idx]; // set incumbant role
	  }
	}
      } // for role_idx;
	
      // Set our own vote. Note number_of_voted trackers (1 or 2) tells us
      // if we are voting ourselves Secondary or Tertiary.
      // Role enums are 0 based, so number_of trackers are 1, 2, 3.
      redundancy_db_obj->getMyTrackerStatePtr()->role= \
	role_array[number_voted_trackers];
      // note: we add our own vote to ourselves when we vote()
      redundancy_db_obj-> getMyTrackerStatePtr()-> \
	votes[number_voted_trackers] = number_voted_trackers;
      redundancy_db_obj->incVotesIn(); //  inc total vote tally
      // no need to wait for next vote or 10 sec if the system had been
      // operational as we are only adding this tracker back.
      redundancy_db_obj->setLateJoiner(true);	
      //redundancy_db_obj->printFullBallot();
      break;

    case VOTE:
    case WAIT_VOTES_IN:
      // go through and extact the vote Primary to Tertiary
      for (int role_idx=0; role_idx<number_voted_trackers; role_idx++) 
	// see who's Guid we are getting a vote for?
	for (int i=0; i<number_voted_trackers; i++)
	  if (guid[role_idx] ==	redundancy_db_obj->getTrackerState_ptr(i)->guid)
	    // increment the vote for tracker based on the role we are checking
	    redundancy_db_obj->getTrackerState_ptr(i)->votes[role_idx]++;
	
      redundancy_db_obj->incVotesIn(); //  inc total vote tally
      break;
      
    case VOTE_RESULTS:
    case STEADY_STATE:
      int db_idx;
      // A vote received after voting is a late joining Tracker. All were
      // interested in is to update our tracker db with the late joiner's role.
      // Verify we have room in our db for a late joining tracker
      // newTracker set in HB reader if we have space
      if (redundancy_db_obj->isNewTracker()) {
	redundancy_db_obj->setNewTracker(false);
	for (int role_idx=0; role_idx<number_voted_trackers; role_idx++)
	  if (guid[role_idx]==source_guid) {
	    // role_idx the role, now find the tracker in the db
	    for (db_idx = 0; db_idx<3; db_idx++)
	      if (redundancy_db_obj->getTrackerState_ptr(db_idx)->guid\
		  == source_guid)
		break;
	    // don't trust thier vote entirely - update the role for the late joining
	    // tacker, not based on it's vote, but upon what role we have available
	    redundancy_db_obj->getTrackerState_ptr(db_idx)->role	\
	      =role_array[redundancy_db_obj->numberOfTrackers()-1];

	    //redundancy_db_obj->printSortedTrackers();
	  }
	// we are not revoting in this case so keep  clear votes clear
	redundancy_db_obj->clearVotes();
      } else {
	std::cerr << "ERROR: Vote from late joining tracker when we have 3 trackers"
		  << std::endl;
      }
      break;
 
    case SHUT_DOWN:
    case ERROR:
    default:// do nothing for these states
      break;
      
    } // if tracker did not vote

  done:
    return;
    
  bad_vote:
    std::cerr << "Vote found inconsistent from source: "
	      << source_guid
	      << std::endl;
    //TODO: Mark that source as failed and ignore heartbeats  
  };
    
} // namespace MODULE
