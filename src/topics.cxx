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
  
  ServoWtr::ServoWtr(
	const dds::domain::DomainParticipant participant,
        bool periodic,

	
        dds::core::Duration period)
    : Writer(participant, "ServoControl", "publisher::servo_topic_writer", periodic, period) {
        // Update Static Topic Data parameters in the beginning of the handler
        // prior to the loop, but after the entity base class creates the sample.
        // std::cout << "Servo Writer C'Tor" << std::endl;
  };

  void ServoWtr::writeData(int32_t x, int32_t y) {
    gimbal.update_pan(x);
    gimbal.update_tilt(y);
    std::cout << "P: " << gimbal.get_pan_position() << \
      "   T: " << gimbal.get_tilt_position() << "\r" << std::flush;
    //this->getMyDataSample()->value<int32_t>("pan", gimbal.get_pan_position());
    //servo_writer->getMyDataSample()->value<int32_t>("tilt", gimbal.get_tilt_position());
    //    servo_writer->write(*this->getMyDataSample());
    if (this->frame_count++ > 10) {
      this->frame_count = 0;
      printf("P: %d T: %d   \r", gimbal.get_pan_position(), gimbal.get_tilt_position());
      fflush(stdout);
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


} // namespace
