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
  enum Roll roll_array[5]{PRIMARY, SECONDARY, TERTIARY, UNASSIGNED, UNASSIGNED};
  std::string roll_string_map[4] {"Primary", "Secondary", "Tertiary", "Not Voted Yet"};
  
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

  
  RedundancyInfo::RedundancyInfo(const dds::domain::DomainParticipant participant)
  {
    // initialize guid tracking array to my_guid followed by 0's
    uint8_t iarr[16] {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    rti::core::Guid ff_guid= convertIArrayToGuid(iarr);
    this->ff_guid = ff_guid; // save this guid
    
    this->array_tracker_states[0].state=OPERATIONAL; // set myself operational   
    this->array_tracker_states[1].guid=ff_guid;
    this->array_tracker_states[2].guid=ff_guid;

    // initialize the ordered array of tracker ptrs
    for (int i=0; i<3; i++)
      ordered_array_tracker_state_ptrs[i]=&array_tracker_states[i];
  }

  
  void RedundancyInfo::printSortedTrackers() {
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

  
  int RedundancyInfo::getMyRollStrength()
  {
    int ownership_strength_roll_map[3] {30,20,10};
    return ownership_strength_roll_map					\
      [this->ordered_array_tracker_state_ptrs[this->my_ordinal-1]->roll];
  }

  void RedundancyInfo::sortSaveGuids()
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

  
  void RedundancyInfo::lostTracker(int tracker_indx)
  {
    // Lost a tracker(tracker_ordinal = tracker_indx+1).
    // We need to promote all lower trackers
    // first clear the trackerStat struct[i]. The trackers in order so we just
    // need to shuffle them forward in the ordered_array_tracker_state_ptrs, and
    // promote thier roll.
    // Note: There must be at least 2 trackers as we are 1 and we lost 1.
    
    // promote all remaining trackers with lower rolls than the one lost
    for (int i=0; i<this->number_of_trackers; i++) {
      // enum Primary 0, Secondary 1, Tertiary 3 
      if (this->ordered_array_tracker_state_ptrs[tracker_indx]->roll <
	  this->ordered_array_tracker_state_ptrs[i]->roll)
	this->ordered_array_tracker_state_ptrs[i]->roll =\
	  roll_array[(this->ordered_array_tracker_state_ptrs[i]->roll)-1];
     }

    // zero out the lost tracker, drop the number of trackers and resort
    this->ordered_array_tracker_state_ptrs[tracker_indx]->guid =\
      this->ff_guid;
    this->ordered_array_tracker_state_ptrs[tracker_indx]->roll = \
      UNASSIGNED;
    this->clearVotesTracker(tracker_indx); // clears Ivoted as well
    this->ordered_array_tracker_state_ptrs[tracker_indx]->state=FAILED;
    this->number_of_trackers--; 
    this->sortSaveGuids(); // force reordering of the array
    this->is_new_tracker = false; // vote null Guids for lost tracker
  }
    
  
  void RedundancyInfo::assessVoteResults(void)
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
	// check for single fault cases (2,1,0) and (2,0,1)
	// mark faulted tracker
	if(this->number_of_trackers==3 && roll_vote == 1) { 
	  this->ordered_array_tracker_state_ptrs[ord]->state=FAILED;
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
			     RedundancyInfo* redundancy_info_obj,
			     bool periodic,
			     dds::core::Duration period) 
    : Writer(participant, "TrackerHeartbeat", \
	     "publisher::tracker_hb_topic_writer",\
	     periodic, period)
  {
    this->my_redundancy_info_obj = redundancy_info_obj;

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
    this->my_redundancy_info_obj->getMyTrackerStatePtr()->guid = guid;
    this->my_redundancy_info_obj->setMyGuid(guid);
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
			     RedundancyInfo* redundancy_info_obj)
    : Reader(participant, "TrackerHeartbeat", "subscriber::tracker_hb_topic_reader")
  {
    this->my_redundancy_info_obj = redundancy_info_obj;
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
    for (int i=0; i<this->my_redundancy_info_obj->numberOfTrackers(); i++)
      if (hb_guid==this->my_redundancy_info_obj->getTrackerState_ptr(i)->guid) {
	 duplicate=true;
	 this->my_redundancy_info_obj->getTrackerState_ptr(i)->hbDeadlineCnt++;
	}

    // if new tracker we have available tracker space, find available slot in
    // tracker array populate it and sort/resort.
    if  (!duplicate && (this->my_redundancy_info_obj->numberOfTrackers() < 3)) {
      this->my_redundancy_info_obj->incNumberOfTrackers();
      for (int i=0; i<3; i++)
	if (this->my_redundancy_info_obj->getTrackerState_ptr(i)->guid == \
	    this->my_redundancy_info_obj->getNullGuid()) {
	  this->my_redundancy_info_obj->getTrackerState_ptr(i)->guid = hb_guid;
	  this->my_redundancy_info_obj->getTrackerState_ptr(i)->state = OPERATIONAL;
	  this->my_redundancy_info_obj->getTrackerState_ptr(i)->hbDeadlineCnt = 1;
	  this->my_redundancy_info_obj->sortSaveGuids();
	  this->my_redundancy_info_obj->setNewTracker(true);
	  break;
	} 
      // std::cout << hb_guid << std::endl;
    }	   
  };


  VoteWtr::VoteWtr(const dds::domain::DomainParticipant participant,
		   RedundancyInfo* redundancy_info_obj,
		   bool periodic,
		   dds::core::Duration period)
    : Writer(participant, "VoteType", \
	     "publisher::vote_topic_writer",\
	     periodic, period)
  {
     this->my_redundancy_info_obj = redundancy_info_obj;
     this->setSampleField("SourceHBwriterHandle", redundancy_info_obj->getMyGuid());
     // initialize all vote samples to NULL guid (so we don't have to worry
     // about accessing an N/A field
     for (int i=0; i<3; i++) {
	this->setSampleField(roll_string_map[i], \
			     my_redundancy_info_obj->getNullGuid());
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
    // We only vote at initialization and anytime we loose a tracker.
    // Redundancy::lostTracker() ensure promotions took place.
    // We know we have the following senarios:
    //    Simplex - {Primary, Unassigned, Unassigned}
    //    Duplex - {Primary, Secondary, Unassigned}
    //    Triplex - {Primary, Secondary, Tertiary}
    //
    // During INITIALIZATION we vote rank order of Guids lowest
    // to highest Primary, Secondary, Tertiary.
    //
    // Operationally, loss of a tracker, we vote for incumbants
    // Current (promoted or now)  Primary / Primary,
    // If Secondary - Current Secondary / Secondary
    // Always fill out null guid vote for empty Secondary
    // and Tertiary (always empty if we are revoting).

    bool was_operational{false};
    bool is_primary {false}, is_secondary{false}, is_tertiary{false};
    int primary_idx, secondary_idx, tertiary_idx, new_tracker_idx;
    
    for (int i=0; i<3; i++){
      //std::cout << my_redundancy_info_obj->getTrackerState_ptr(i)->guid << " : "
      //           << my_redundancy_info_obj->getTrackerState_ptr(i)->roll;
      
      switch(this->my_redundancy_info_obj->getTrackerState_ptr(i)->roll) {
      case UNASSIGNED:
	// a new tracker does not yet have a roll
	new_tracker_idx = i;
	break;
       case PRIMARY:
	was_operational = true;
	is_primary = true;
	primary_idx = i;
	break;
      case SECONDARY:
	was_operational = true;
	is_secondary = true;
	secondary_idx = i;
	break;
      case TERTIARY:
	was_operational = true;	
	is_tertiary = true;
	tertiary_idx=1;
	break;
      default:
	std::cerr << "switch assessVote error and abort" << std::endl;
      }
    } // for

    // Set the number of trackers we are voting for (non-null guids)
    this->getMyDataSample()->value("NumberOfTrackers", \
				    my_redundancy_info_obj->numberOfTrackers());
    if (was_operational) {
      std::cout << "Operational: "
		<< my_redundancy_info_obj->getMyOrdinal()
		<< " " << primary_idx << " " << secondary_idx
		<< " " << tertiary_idx << std::endl;
      if (is_primary) {// and was a Primary so keep incumbant in office
	this->setSampleField("Primary",
		   my_redundancy_info_obj->getTrackerState_ptr(primary_idx)->guid);
	if (is_secondary)  // and was secondary keep incumbant secondary
	  this->setSampleField("Secondary",
		   my_redundancy_info_obj->getTrackerState_ptr(secondary_idx)->guid);
	else  // no secondary, primary only fill out with null guid
	    this->setSampleField("Secondary", my_redundancy_info_obj->getNullGuid());
	// always fill out three votes, so set Tertiary field null guid
	this->setSampleField("Tertiary", my_redundancy_info_obj->getNullGuid());

	if (is_tertiary) // was operation w/failure but we have 3 Trackers
	  std::cerr << " ERROR: Voting - Inconsistant state " << std::endl;
	
      }else // No Primary: tracker loss() would have ensured a promotion to primary
	  std::cerr << " ERROR Voting: Was operation w/No Primary" << std::endl;
      
    } else { // was not operational, no rolls ever assigned
      // vote by sorted guid, lowest {Primary} to higher 
      for (int i=0; i<3; i++) 
	this->setSampleField(roll_string_map[i],			\
			   my_redundancy_info_obj->getTrackerState_ptr(i)->guid);
    }

    // We've populated the Vote Topic with our Selections according to the voting algorithm.
    // Now Read the topic back and register our vote
    for (int l=0; l<my_redundancy_info_obj->numberOfTrackers(); l++) {

      std::vector<uint8_t> seq_values_p = getMyDataSample()->get_values<uint8_t>(roll_string_map[l]);

      uint8_t iarr[16];
      for (int i=15; i>=0; i--) {
	iarr[i]=seq_values_p.back();
	seq_values_p.pop_back();
      }

      rti::core::Guid guid= convertIArrayToGuid(iarr);
      // update my vote array based on the roll (i)
      for (int i=0; i<my_redundancy_info_obj->numberOfTrackers(); i++)
	if (guid == my_redundancy_info_obj->getTrackerState_ptr(i)->guid)
	  my_redundancy_info_obj->getTrackerState_ptr(i)->votes[i]++;
	
    } // for
  
    // cast our vote - write the vote sample
    this->topicWriter.write(*this->getMyDataSample());
  }

    
  VoteRdr::VoteRdr(const dds::domain::DomainParticipant participant,
		   RedundancyInfo* redundancy_info_obj)
    : Reader(participant, "VoteType", "subscriber::vote_topic_reader")
  {
    this->my_redundancy_info_obj = redundancy_info_obj;    
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
    std::cout << "Received Vote" << std::endl;

    rti::core::Guid this_guid, guid[3];
    bool tracker_voted {false};

    int number_voted_trackers;

    // Only process one durable vote. Tthis code is not multi-entrant and
    // we don't need the second consistent vote.
    //  
    if (my_redundancy_info_obj->voteRdrLocked()) { // needs to be atomic
      std::cout << "Vote Reader Locked" << std::endl;
      goto done;
    }

    tracker_voted = false;
    number_voted_trackers  = (int)data.value<int32_t>("NumberOfTrackers");

    // verify this tracker has not already voted (should be impossible)
    this_guid = this->extractGuid(data, "SourceHBwriterHandle");
    for (int i=0; i<number_voted_trackers; i++) {
      if (this_guid == \
	  my_redundancy_info_obj->getTrackerState_ptr(i)->guid) {
	if (my_redundancy_info_obj->getTrackerState_ptr(i)->Ivoted)
	  tracker_voted = true;
	else // if found guid and not voted, mark it as voted
	  my_redundancy_info_obj->getTrackerState_ptr(i)->Ivoted = true;
	break;
      }
    }
    if (!tracker_voted) { // process vote 
      // first check the vote is consistent - all guids are different
      // for each roll
      for (int l=0; l<number_voted_trackers; l++) {
	guid[l] = this->extractGuid(data, roll_string_map[l]);
	if (l>1)
	  for (int k=0; k<l; k++)
	    if (guid[k]==guid[l])
	      goto bad_vote;
      }
      // Votes are durable, so if we get a vote in the INITIALIZE state,
      // we receive votes in one of up to 3 states:
      //   INITIALIZE - indicates the system has been running, other trackers
      //   (1 or 2 of them) have voted and we are a late joiners.
      //   In this case we should simply make our view of received votes based
      //   on the first vote we receive
      //   VOTE - heartbeats and durable votes can race eachother. This covers
      //   the case where heartbeats beat durable votes and a late joiner
      //    quickly went to VOTE state
      //   WAIT_VOTES_IN - indicates this tracker came up with the system,
      //   so tally each vote and vote once all trackers we learned about
      //   during init state are in.
      if (my_redundancy_info_obj->smState()==INITIALIZE || \
	  my_redundancy_info_obj->smState()==PREVOTE) {

	for (int roll_idx=0; roll_idx<number_voted_trackers; roll_idx++) {
	  // see who's Guid we are getting a vote for?
	  for (int i=0; i<number_voted_trackers; i++) {
	    if (guid[roll_idx] ==						\
		my_redundancy_info_obj->getTrackerState_ptr(i)->guid) {
	      // increment the vote for tracker based on the roll we are checking
	      // and set this tracker OPERATIONAL as we received a transient
	      // local vote from it's vote Writer. Also set the HB cnt so we
	      // don't immediately declare it missing.
	      std::cout << "Placeing votes for Tracker: "
			<< guid[roll_idx] << " "
			<< roll_string_map[roll_idx]
			<< " votes: " << number_voted_trackers+1;
	      my_redundancy_info_obj->getTrackerState_ptr(i)->votes[roll_idx] \
		= number_voted_trackers+1; // add my vote to the same
	      my_redundancy_info_obj->getTrackerState_ptr(i)->state = OPERATIONAL;
	      my_redundancy_info_obj->getTrackerState_ptr(i)->hbDeadlineCnt = 1;
	    }
	  }
	} // for roll_idx;
	std::cout << "System Has been running - use old votes" << std::endl;
	// Set our own vote. Note number_of_voted trackers (1 or 2) tells us
	// if we are voting ourselves Secondary or Tertiary.
	// Roll enums are 0 based, so number_of trackers are 1, 2, 3.
	my_redundancy_info_obj->getMyTrackerStatePtr()->roll= \
	  roll_array[number_voted_trackers];
	my_redundancy_info_obj-> getMyTrackerStatePtr()-> \
	  votes[number_voted_trackers] = number_voted_trackers+1;
	// no need to wait for next vote or 10 sec if the system had been
	// operational as we are only adding this tracker back.
	my_redundancy_info_obj->setLateJoiner(true);	
	my_redundancy_info_obj->setVotesExpected(number_voted_trackers+1);

      } else { // the system was not previously running
        std::cout << "System just up process new vote - " << std::endl;
	my_redundancy_info_obj->incVotesIn(); //  inc total vote tally
	// go through and extact the vote Primary to Tertiary
	for (int roll_idx=0; roll_idx<number_voted_trackers; roll_idx++) {
	  // see who's Guid we are getting a vote for?
	  for (int i=0; i<number_voted_trackers; i++) {
	    if (guid[roll_idx] ==						\
		my_redundancy_info_obj->getTrackerState_ptr(i)->guid)
	      // increment the vote for tracker based on the roll we are checking
	      my_redundancy_info_obj->getTrackerState_ptr(i)->votes[roll_idx]++;
	  }
	} // for roll_idx;
      }
    } // if tracker did not vote
    my_redundancy_info_obj->clrVoteRdrLock();
  done:
    return;
    
  bad_vote:
    std::cerr << "Vote found inconsistent" << std::endl;
  };
    
} // namespace MODULE

