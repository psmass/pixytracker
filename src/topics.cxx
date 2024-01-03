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

  /* Helper functions to covert between InstanceHandles, GUIDs and arrays
   */
  
  // need to covert Instance Handles to Guids to use the math operators to compare values
  rti::core::Guid convertToGuid ( const dds::core::InstanceHandle& instanceHandle ) {
    rti::core::Guid guid;
    memcpy(&guid,reinterpret_cast<DDS_Octet const *>(&instanceHandle ),16);
    return guid;
  }

  // test routine to create a Guid to try comparison operators
  rti::core::Guid convertIArrayToGuid ( const uint8_t *array ) {
    rti::core::Guid guid;
    memcpy(&guid,array,16);
    return guid;
  }

  // routine to put Instance Handle in an array of 8 bit ints
  void convertInstanceHandleToIArray (uint8_t *array, const dds::core::InstanceHandle& instanceHandle ) {
    memcpy(array,reinterpret_cast<DDS_Octet const *>(&instanceHandle),16);
  }

  // routine to put Guid in an array of 8 bit ints
  void convertGuidToIArray (uint8_t *array, rti::core::Guid guid ) {
    memcpy(array,reinterpret_cast<DDS_Octet const *>(&guid),16);
  }

  
  RedundancyInfo::RedundancyInfo(const dds::domain::DomainParticipant participant) {
    /* Get my participant Instance Handle and send it rather than Guids.
       We'll need to convert instance handles to GUIDS to use the math operators. 
       Note sure how to get Guid directly and can't do math on Instance handle.
     */
    const dds::core::InstanceHandle handle=participant->instance_handle();
    std::cout << "INSTANCE HANDLE: " << handle << std::endl;

    this->my_p_guid  = convertToGuid(handle);
    std::cout << "GUID Convert: " << my_p_guid << std::endl;

    // initialize guid tracking array to my_guid followed by 0's
    uint8_t iarr[16] {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    rti::core::Guid ff_guid= convertIArrayToGuid(iarr);
    this->ordered_array_p_guids[0]=my_p_guid;
    this->ordered_array_p_guids[1]=ff_guid;
    this->ordered_array_p_guids[2]=ff_guid;

    my_ordinal = 1; // initially 1, changes when we receive durable vote or heartbeats

  }

  void RedundancyInfo::sortSaveHbGuid(rti::core::Guid hb_guid)
  {
    // first make sure we don't already have this tracker
    bool dup_guid {false};
    for (int i=0; i<3; i++) {
      if (this->ordered_array_p_guids[i]==hb_guid){
	dup_guid = true;
	break;
      }
    }

    if (!dup_guid) {
      // This routine is only called if we have << 3 trackers
      // place on the end of the zero-based array and bubble sort
      this->ordered_array_p_guids[this->number_of_trackers] = hb_guid;
      this->number_of_trackers++; // at least two now

      rti::core::Guid temp_guid;
      for (int l=0; l<this->number_of_trackers-1; l++) {
	
	for (int i=0; i<this->number_of_trackers-1; i++) {
	  if (this->ordered_array_p_guids[i] > this->ordered_array_p_guids[i+1]) {
	    // swap them
	    temp_guid = this->ordered_array_p_guids[i];
	    this->ordered_array_p_guids[i] = this->ordered_array_p_guids[i+1];
	    this->ordered_array_p_guids[i+1] = temp_guid;
	    // now adjust the ordinal if the moved tracker is mine/self
	    if (temp_guid == this->my_p_guid) 
	      this->my_ordinal = i+1;
	  }
	}
      }
      std::cout << "DISCOVERED SORTED TRACKERS " << std::endl;
      for (int i=0; i<3; i++) 
	std::cout << this->ordered_array_p_guids[i] << std::endl;
			      
    } // if dup
  }
    
  ServoWtr::ServoWtr(
	const dds::domain::DomainParticipant participant,
        bool periodic,	
        dds::core::Duration period)
    : Writer(participant, "ServoControl", "publisher::servo_topic_writer", periodic, period) {
        // Update Static Topic Data parameters in the beginning of the handler
        // prior to the loop, but after the entity base class creates the sample.
        // std::cout << "Servo Writer C'Tor" << std::endl;
    this->getMyDataSample()->value<uint16_t>("frequency", SERVO_FREQUENCY_HZ);
  };

  void ServoWtr::writeData(int32_t x, int32_t y) {
    gimbal.update_pan(x);
    gimbal.update_tilt(y);
    this->getMyDataSample()->value<uint16_t>("pan", gimbal.get_pan_position());
    this->getMyDataSample()->value<uint16_t>("tilt", gimbal.get_tilt_position());
    this->topicWriter.write(*this->getMyDataSample());
    if (this->frame_count++ > 10) {
      this->frame_count = 0;
      std::cout << "P: " << gimbal.get_pan_position() \
		<<" T: " << gimbal.get_tilt_position() << "          ""\r" << std::flush;
    }
  };

  
  ShapesRdr::ShapesRdr(const dds::domain::DomainParticipant participant,  ServoWtr* servoWriter)
    : Reader(participant, "ShapeTypeExtended", "subscriber::shape_topic_reader") {
    
    this->servo_writer = servoWriter;
    
  };

  void ShapesRdr::handler(dds::core::xtypes::DynamicData& data) {
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
			     RedundancyInfo redundancy_info_obj,
			     bool periodic,
			     dds::core::Duration period) 
    : Writer(participant, "TrackerHeartbeat", \
	     "publisher::tracker_hb_topic_writer",\
	     periodic, period) {

    this->my_redundancy_info_obj = &redundancy_info_obj;

    // get my guid and place it in the heartbeat sample
    uint8_t iarr[16];
    convertGuidToIArray(iarr, redundancy_info_obj.getMyGuid());

    std::vector<uint8_t> seq_values;
    for (int i=0; i<16; i++)
      seq_values.push_back(iarr[i]);
    
    this->getMyDataSample()->set_values("MyParticipantHandle", seq_values);
 
  }
  
    
  // write() is effectively a runtime down cast for periodic data
  void HeartbeatWtr::write(void) {
    this->topicWriter.write(*this->getMyDataSample());
  };
  

  HeartbeatRdr::HeartbeatRdr(const dds::domain::DomainParticipant participant,
			     RedundancyInfo redundancy_info_obj)
    : Reader(participant, "TrackerHeartbeat", "subscriber::tracker_hb_topic_reader")
  {
    this->my_redundancy_info_obj = &redundancy_info_obj;
  };

  void HeartbeatRdr::handler(dds::core::xtypes::DynamicData& data) {
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
	

    /*
    // Test Guid Math operators - seems to work fine
    const uint8_t i[16] {7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4};
    std::cout << "String: " << i << std::endl;
    rti::core::Guid myMadeupGuid = convertIArrayToGuid(i);
    std::cout << "TestGuid: " << myMadeupGuid << std::endl;

    if (myMadeupGuid > hbGuid)
      std::cout << "Madeup Guid is larger" << std::endl;
    else
      std::cout << "myGuid is larger" << std::endl;
    */
    
  };


  VoteWtr::VoteWtr(const dds::domain::DomainParticipant participant,
		   RedundancyInfo redundancy_info_obj,
		   bool periodic,
		   dds::core::Duration period)
    : Writer(participant, "VoteType", \
	     "publisher::vote_topic_writer",\
	     periodic, period)
  {
     this->my_redundancy_info_obj = &redundancy_info_obj;
  }

  VoteRdr::VoteRdr(const dds::domain::DomainParticipant participant,
		   RedundancyInfo redundancy_info_obj)
    : Reader(participant, "VoteType", "subscriber::vote_topic_reader")
  {
    this->my_redundancy_info_obj = &redundancy_info_obj;    
  }

  void VoteRdr::handler(dds::core::xtypes::DynamicData& data)
  {
    std::cout << "Received Vote" << std::endl;
  }
    
} // namespace MODULE

