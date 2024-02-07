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

  void ShapesRdr::handler(rti::sub::LoanedSample<rti::core::xtypes::DynamicDataImpl>* sample)
  {
    DDS_ReturnCode_t retcode;
    // Control the pan & til
    x = sample->data().value<int32_t>("x");
    y = sample->data().value<int32_t>("y");
    this->servo_writer->writeData(x,y);

  };

} // namespace MODULE

