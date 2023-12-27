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

#include <iostream>
#include <chrono>
#include "gimbal.hpp"

namespace GIMBAL
{
  Gimbal::Gimbal() {
    pan.position        = PIXY_RCS_CENTER_POS;
    pan.previous_error     = 0x80000000;
    pan.proportional_gain  = PAN_PROPORTIONAL_GAIN;
    pan.derivative_gain    = PAN_DERIVATIVE_GAIN;
    tilt.position          = PIXY_RCS_CENTER_POS;
    tilt.previous_error    = 0x80000000;
    tilt.proportional_gain = TILT_PROPORTIONAL_GAIN;
    tilt.derivative_gain   = TILT_DERIVATIVE_GAIN;
  }

  void Gimbal::update_pan (DDS_Long x) {
    int pan_error = PIXY_X_CENTER - x;
    update(&pan, pan_error);
  }

  void Gimbal::update_tilt (DDS_Long y) {
    int tilt_error = y - PIXY_Y_CENTER;
    update(&tilt, tilt_error);
  }

  int32_t Gimbal::get_pan_position()  { return pan.position; }
  int32_t Gimbal::get_tilt_position() { return tilt.position; }


  void Gimbal::update(struct GimbalParams * gimbal, int32_t error)
  {
    long int velocity;
    int32_t  error_delta;
    int32_t  P_gain;
    int32_t  D_gain;

    if(gimbal->previous_error != 0x80000000)
      {
	error_delta = error - gimbal->previous_error;
	P_gain      = gimbal->proportional_gain;
	D_gain      = gimbal->derivative_gain;

	/* Using the proportional and derivative gain for the gimbal,
	   calculate the change to the position.  */
	velocity = (error * P_gain + error_delta * D_gain) >> 10;
	gimbal->position += velocity;

	if (gimbal->position > PIXY_RCS_MAX_POS)
	  {
	    gimbal->position = PIXY_RCS_MAX_POS;
	  } else if (gimbal->position < PIXY_RCS_MIN_POS)
	  {
	    gimbal->position = PIXY_RCS_MIN_POS;
	  }
      }
    gimbal->previous_error = error;
  }

} //namespace GIMBAL
