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
  rti::core::Guid convertIntToGuid ( const int32_t& i ) {
    rti::core::Guid guid;
    memcpy(&guid,reinterpret_cast<DDS_Octet const *>(&i ),16);
    return guid;
  }

  // test routine to create a Guid to try comparison operators
  void convertInstanceHandleToArray (unsigned char *guidArray, const dds::core::InstanceHandle& instanceHandle ) {
    memcpy(guidArray,reinterpret_cast<DDS_Octet const *>(&instanceHandle ),16);
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
    : Writer(participant, "ControllerHeartbeat", "publisher::controller_hb_topic_writer", periodic, period) {

    using namespace dds::core::xtypes;
    using namespace rti::core::xtypes;
    
    /* interesting ways to get the participant handle 
    dds::domain::qos::DomainParticipantQos p_qos;
    p_qos=participant->qos();

    std::cout <<"Participant QoS " << p_qos << std::endl;

    rti::core::policy::WireProtocol wc;
    int32_t p_id=p_qos->wire_protocol.participant_id();
    int32_t h_id=p_qos->wire_protocol.rtps_host_id();
    int32_t a_id=p_qos->wire_protocol.rtps_app_id();
    int32_t i_id=p_qos->wire_protocol.rtps_instance_id();
    
    std::cout << "P:H:A:I ID:" << p_id << " " << h_id << " " << a_id << " " << i_id << std::endl;
    */

    /* Get my participant Instance Handle and send it rather than Guids.
       We'll need to convert instance handles to GUIDS to use the math operators. 
       Note sure how to get Guid directly and can't do math on Instance handle.
     */
    const dds::core::InstanceHandle handle=participant->instance_handle();
    std::cout << "INSTANCE HANDLE: " << handle << std::endl;

    rti::core::Guid myGuid = convertToGuid(handle);
    std::cout << "GUID Convert: " << myGuid << std::endl;

    /* // Test Guid Math operators - seems to work fine

    const int32_t i = 20082004;
    rti::core::Guid myMadeupGuid = convertIntToGuid(i);

    if (myMadeupGuid > myGuid)
      std::cout << "Madeup Guid is larger" << std::endl;
    else
      std::cout << "myGuid is larger" << std::endl;

    std::cout << "GUID Int Convert: " << myMadeupGuid << std::endl;
    */
  
    unsigned char arr[16];
    convertInstanceHandleToArray(arr, handle);
  
    // Attempt to load the sample using this ref (did not seem to work):
    // https://community.rti.com/examples/dynamic-data-accessing-sequence-members
    DynamicData my_participant_handle = \
      this->getMyDataSample()->value<DynamicData>("MyParticipantHandle");

    DynamicType my_participant_handle_type = \
      static_cast<const SequenceType &>(my_participant_handle.type()).content_type();
    

    std::cout << "Participant Char Array:";
    for (int i=0; i<16; i++) {
      printf("0x%x ", arr[i]);
      //LoanedDynamicData loaned_data = my_participant_handle.loan_value(i+1);
      //loaned_data.get().value("MyParticipantHandle",(int8_t)arr[i]);
      //loaned_data.return_loan();
      //DynamicData my_participant_handle_element(my_participant_handle_type);
      //my_participant_handle.value(i+1, arr[i]);

    }
    std::cout << std::endl;
    //this->getMyDataSample()->value("MyParticipantHandle", my_participant_handle);
    
    // Brute force way to load the handle into the sample
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[0]", (uint8_t)arr[0]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[1]", (uint8_t)arr[1]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[2]", (uint8_t)arr[2]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[3]", (uint8_t)arr[3]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[4]", (uint8_t)arr[4]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[5]", (uint8_t)arr[5]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[6]", (uint8_t)arr[6]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[7]", (uint8_t)arr[7]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[8]", (uint8_t)arr[8]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[9]", (uint8_t)arr[9]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[10]", (uint8_t)arr[10]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[11]", (uint8_t)arr[11]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[12]", (uint8_t)arr[12]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[13]", (uint8_t)arr[13]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[14]", (uint8_t)arr[14]);
    this->getMyDataSample()->value<uint8_t>("MyParticipantHandle[15]", (uint8_t)arr[15]);
    
  }
  
    
  // write() is effectively a runtime down cast for periodic data
  void HeartbeatWtr::write(void) {
    this->topicWriter.write(*this->getMyDataSample());
  };
  

  HeartbeatRdr::HeartbeatRdr(const dds::domain::DomainParticipant participant)
    : Reader(participant, "ControllerHeartbeat", "subscriber::controller_hb_topic_reader"){
  };

  void HeartbeatRdr::handler(dds::core::xtypes::DynamicData& data) {
  };
        


} // namespace
