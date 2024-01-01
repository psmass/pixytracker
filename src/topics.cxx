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
    memcpy(array,reinterpret_cast<DDS_Octet const *>(&instanceHandle ),16);
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
      //printf("P: %d T: %d   \r", gimbal.get_pan_position(), gimbal.get_tilt_position());
      //fflush(stdout);
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
	     bool periodic,
	     dds::core::Duration period)
    : Writer(participant, "ControllerHeartbeat", \
	     "publisher::controller_hb_topic_writer",\
	     periodic, period) {

    /* Get my participant Instance Handle and send it rather than Guids.
       We'll need to convert instance handles to GUIDS to use the math operators. 
       Note sure how to get Guid directly and can't do math on Instance handle.
     */
    const dds::core::InstanceHandle handle=participant->instance_handle();
    std::cout << "INSTANCE HANDLE: " << handle << std::endl;

    rti::core::Guid myGuid = convertToGuid(handle);
    std::cout << "GUID Convert: " << myGuid << std::endl;

    uint8_t iarr[16];
    convertInstanceHandleToIArray(iarr, handle);

    std::vector<uint8_t> seq_values;
    for (int i=0; i<16; i++)
      seq_values.push_back(iarr[i]);

    this->getMyDataSample()->set_values("MyParticipantHandle", seq_values);
   
  }
  
    
  // write() is effectively a runtime down cast for periodic data
  void HeartbeatWtr::write(void) {
    this->topicWriter.write(*this->getMyDataSample());
  };
  

  HeartbeatRdr::HeartbeatRdr(const dds::domain::DomainParticipant participant)
    : Reader(participant, "ControllerHeartbeat", "subscriber::controller_hb_topic_reader")
    {
  };

  void HeartbeatRdr::handler(dds::core::xtypes::DynamicData& data) {
    std::vector<uint8_t> seq_values = data.get_values<uint8_t>("MyParticipantHandle");

    std::cout << "Received Heartbeat: GUID=";
    
    uint8_t iarr[16];
    for (int i=15; i>=0; i--) {
      iarr[i]=seq_values.back();
      seq_values.pop_back();
    }

    rti::core::Guid hbGuid= convertIArrayToGuid(iarr);
    std::cout << hbGuid << std::endl;

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
			bool periodic,
			dds::core::Duration period)
    : Writer(participant, "VoteType", \
	     "publisher::vote_topic_writer",\
	     periodic, period)
  {
  }

  VoteRdr::VoteRdr(const dds::domain::DomainParticipant participant)
    : Reader(participant, "VoteType", "subscriber::vote_topic_reader")
  {
  }

  void VoteRdr::handler(dds::core::xtypes::DynamicData& data)
  {
    std::cout << "Received Vote" << std::endl;
  }
    
} // namespace MODULE

