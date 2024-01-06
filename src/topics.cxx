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

namespace MODULE
{

  // Array to Index map enum Roll
  enum Roll roll_array[4]{PRIMARY, SECONDARY, TERTIARY, UNASSIGNED};
  std::string roll_string_map[3] {"Primary", "Secondary", "Tertiary"};
  
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
    /* Get my participant Instance Handle and send it rather than Guids.
       We'll need to convert instance handles to GUIDS to use the math operators. 
       Note sure how to get Guid directly and can't do math on Instance handle.
     */
    const dds::core::InstanceHandle handle=participant->instance_handle();
    // std::cout << "INSTANCE HANDLE: " << handle << std::endl;

    this->array_tracker_states[0].guid  = convertToGuid(handle);
    std::cout << "GUID Convert: " << this->array_tracker_states[0].guid  << std::endl;

    // initialize guid tracking array to my_guid followed by 0's
    uint8_t iarr[16] {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    rti::core::Guid ff_guid= convertIArrayToGuid(iarr);
    this->array_tracker_states[0].state=OPERATIONAL; // set myself operational   
    this->array_tracker_states[1].guid=ff_guid;
    this->array_tracker_states[2].guid=ff_guid;

    // initialize the ordered array of tracker ptrs
    for (int i=0; i<3; i++)
      ordered_array_tracker_state_ptrs[i]=&array_tracker_states[i];

  }

  
  int RedundancyInfo::getMyRollStrength()
  {
    int ownership_strength_roll_map[3] {30,20,10};
    return ownership_strength_roll_map					\
      [this->ordered_array_tracker_state_ptrs[this->my_ordinal-1]->roll];
  }


  void RedundancyInfo::sortSaveHbGuid(rti::core::Guid hb_guid)
  {

    // first make sure we don't already have this tracker
    bool dup_guid {false};
    for (int i=0; i<3; i++) {
      if (this->array_tracker_states[i].guid==hb_guid){
	dup_guid = true;
	break;
      }
    }

    if (!dup_guid) {
      // This routine is only called if we have < 3 trackers
      // place on the end of the zero-based array and bubble sort the pointers
      this->array_tracker_states[this->number_of_trackers].guid = hb_guid;
      this->number_of_trackers++; // at least two now

      TrackerState* temp_tracker_state_ptr;
      for (int l=0; l<this->number_of_trackers-1; l++) {
	
	for (int i=0; i<this->number_of_trackers-1; i++) {
	  if (this->ordered_array_tracker_state_ptrs[i]->guid >
	      this->ordered_array_tracker_state_ptrs[i+1]->guid) {
	    // we are going to swap the pointers to order the pointer array
	    temp_tracker_state_ptr = this->ordered_array_tracker_state_ptrs[i];
	    if (i == this->my_ordinal-1) { // we are about to bubble ourselves, adjust ordinal
	      this->my_ordinal = i+2;  // remember ordinals are 1 based
	    }
	    this->ordered_array_tracker_state_ptrs[i] = \
	      this->ordered_array_tracker_state_ptrs[i+1];
	    this->ordered_array_tracker_state_ptrs[i+1] = temp_tracker_state_ptr;
	  }
	}
      }
      /*
      std::cout << "DISCOVERED SORTED TRACKERS " << std::endl;
      for (int i=0; i<3; i++) 
	std::cout << this->ordered_array_tracker_state_ptrs[i]->guid
		  << std::endl;
      */		      
    } // if dup
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
    std::cout << "This Tracker was unanomously voted Roll: "
	      << roll_string_map[this->ordered_array_tracker_state_ptrs[my_ordinal-1]->roll]
	      << std::endl;
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

    // get my guid and place it in the heartbeat sample
    uint8_t iarr[16];
    convertGuidToIArray(iarr, redundancy_info_obj->getMyGuid());

    std::vector<uint8_t> seq_values;
    for (int i=0; i<16; i++)
      seq_values.push_back(iarr[i]);
    
    this->getMyDataSample()->set_values("MyParticipantHandle", seq_values);
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
    //std::cout << "Received Heartbeat: GUID=";

    // we only assess a guid from a heartbeat if we have available tracker space
    if (this->my_redundancy_info_obj->numberOfTrackers() < 3) {
      
      std::vector<uint8_t> seq_values = data.get_values<uint8_t>("MyParticipantHandle");
    
      uint8_t iarr[16];
      for (int i=15; i>=0; i--) {
	iarr[i]=seq_values.back();
	seq_values.pop_back();
      }

      rti::core::Guid hbGuid= convertIArrayToGuid(iarr);
      this->my_redundancy_info_obj->sortSaveHbGuid(hbGuid);    
    
      //std::cout << hbGuid << std::endl;
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
     this->setSampleField("SourceParticipantHandle", redundancy_info_obj->getMyGuid());
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
    // first is there already a primary and secondary? Existing rolls
    // take presidence over votes
    // note: if we were a new controller, we sat in the INITIALIZE to get heartbeats
    // form other controllers or 10 sec. We also would have populated the Redundancy
    // state object from durable vote topics if the system had be operational (i.e.
    // other Trackers had voted previously and were either primary or secondary.

    bool was_operational{false}, is_primary {false}, is_secondary{false}, is_tertiary{false};
    int primary_idx, secondary_idx, tertiary_idx;
    
    for (int i=0; i<3; i++){
      //std::cout << my_redundancy_info_obj->getTrackerState_ptr(i)->guid << " : "
      //           << my_redundancy_info_obj->getTrackerState_ptr(i)->roll;
      
      switch(this->my_redundancy_info_obj->getTrackerState_ptr(i)->roll) {
      case UNASSIGNED:
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
    // At this point, durable vote topics would have populated the trackerState
    // object. Hense how we knew was_operational and if we have primary, secondary etc.
    
    if (was_operational) {
      if (is_primary) {// and was a Primary so keep incumbant in office
	this->setSampleField("Primary",
			     my_redundancy_info_obj->getTrackerState_ptr(primary_idx)->guid);
	if (is_secondary) { // and was secondary keep incumbant secondary
	this->setSampleField("Secondary",
			     my_redundancy_info_obj->getTrackerState_ptr(secondary_idx)->guid);
	// and vote ourselves to Tertiary
	this->setSampleField("Tertiary",
			     my_redundancy_info_obj->			\
			     getTrackerState_ptr(my_redundancy_info_obj->getMyOrdinal())->guid);
	}
			     			    
      } else if (is_secondary) { // no primary, so secondary must be running the show
	// vote to promote secondary
	this->setSampleField("Primary",
			     my_redundancy_info_obj->getTrackerState_ptr(secondary_idx)->guid);
	if (is_tertiary) { // should be since single fault, must be primary failed.
	  this->setSampleField("Secondary",
			       my_redundancy_info_obj->getTrackerState_ptr(secondary_idx)->guid);
	  // and vote ourselves to Tertiary
	  this->setSampleField("Tertiary",\
			       my_redundancy_info_obj-> \
			       getTrackerState_ptr(my_redundancy_info_obj->getMyOrdinal())->guid);
	}	  

      } else if (is_tertiary) { // theoretically a double fault if teritary is running the show
	this->setSampleField("Primary",\
			     my_redundancy_info_obj->getTrackerState_ptr(tertiary_idx)->guid);
	// and vote ourselves to Secondary
	this->setSampleField("Secondary",\
			     my_redundancy_info_obj-> \
			     getTrackerState_ptr(my_redundancy_info_obj->getMyOrdinal())->guid);
      }  else { // no one running the show, but we were running
	std::cerr << " Voting ambiguity, System was operational but no rolls assigned "
		  << std::endl;
      }
    } else { // was not operational, no rolls ever assigned
      // vote by sorted guid, lowest {Primary} to higher 
      for (int i=0; i<this->my_redundancy_info_obj->numberOfTrackers(); i++) {
	this->setSampleField(roll_string_map[i], \
			     my_redundancy_info_obj->getTrackerState_ptr(i)->guid);
      }

      // We've populated the Vote Topic with our Selections according to the voting algorithm.
      // Now Read the topic back an register our vote
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
	
	/*
	
        if (guid == my_redundancy_info_obj->getTrackerState_ptr(my_redundancy_info_obj->getMyOrdinal()-1)->guid)
	  {
	    my_redundancy_info_obj->\
	      getTrackerState_ptr(my_redundancy_info_obj->getMyOrdinal()-1)->roll=roll_array[i];
	    my_redundancy_info_obj->\
	      getTrackerState_ptr(my_redundancy_info_obj->getMyOrdinal()-1)->votes[i]++;

	    std::cout << " I Registered votes for myself: "
		      <<  my_redundancy_info_obj->getTrackerState_ptr \
	                          (my_redundancy_info_obj->getMyOrdinal()-1)->guid
		      << " Index: " << i
		      << " Roll: " << roll_string_map[i]
		      << " Vote: " <<  my_redundancy_info_obj-> \
	      getTrackerState_ptr(my_redundancy_info_obj->getMyOrdinal()-1)->votes[i] << std::endl;

	  }
	*/
      } // for
  
    }
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
    
    // verify this tracker has not already voted (should be impossible)
    this_guid = this->extractGuid(data, "SourceParticipantHandle");
    for (int i=0; i<my_redundancy_info_obj->numberOfTrackers(); i++) {
      if (this_guid == \
	  my_redundancy_info_obj->getTrackerState_ptr(i)->guid \
	  && \
	  my_redundancy_info_obj->getTrackerState_ptr(i)->Ivoted) {
	tracker_voted = true;
	break;
      }
    }

    if (!tracker_voted) {
      // first check the vote is consistent - all guids are different
      // for each roll.
      for (int l=0; l<my_redundancy_info_obj->numberOfTrackers(); l++) {	
	guid[l] = this->extractGuid(data, roll_string_map[l]);
	if (l>1)
	  for (int k=0; k<l; k++)
	    if (guid[k]==guid[l])
	      goto bad_vote;
       }
			  
      my_redundancy_info_obj->incVotesIn(); // mark this tracker as voted
      // go through and extact the vote Primary to Tertiary
      for (int roll_idx=0; roll_idx<my_redundancy_info_obj->numberOfTrackers(); roll_idx++) {
        // see who's Guid we are getting a vote for?
	for (int i=0; i<my_redundancy_info_obj->numberOfTrackers(); i++) {
	  if (guid[roll_idx] ==						\
	      my_redundancy_info_obj->getTrackerState_ptr(i)->guid)
	    // increment the vote for tracker based on the roll we are checking
	    my_redundancy_info_obj->getTrackerState_ptr(i)->votes[roll_idx]++;
	  }
      } // for roll_idx;
    } // if tracker did not vote
    return;
    
  bad_vote:
    std::cerr << "Vote found inconsistent" << std::endl;
  };
    
} // namespace MODULE

