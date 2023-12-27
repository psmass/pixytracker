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

#ifndef GIMBAL_HPP
#define GIMBAL_HPP

#include <dds/dds.hpp>

namespace GIMBAL
{


#define NUM_SIGS 7

#define INDEX_RED  0
#define INDEX_ORANGE 1
#define INDEX_YELLOW 2
#define INDEX_GREEN 3
#define INDEX_CYAN 4
#define INDEX_BLUE 5
#define INDEX_PURPLE 6

#define S0_LOWER_LIMIT -200
#define S0_UPPER_LIMIT 200
#define S1_LOWER_LIMIT -200
#define S1_UPPER_LIMIT 200
#define SERVO_FREQUENCY_HZ 60

// Pixy x-y position values
 #define PIXY_MIN_X                  0
 #define PIXY_MAX_X                  319
 #define PIXY_MIN_Y                  0
 #define PIXY_MAX_Y                  199

 // RC-servo values
 #define PIXY_RCS_MIN_POS            0
 #define PIXY_RCS_MAX_POS            1000
 #define PIXY_RCS_CENTER_POS         ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/2)

// PID control parameters //
//#define PAN_PROPORTIONAL_GAIN     400	// 400 350
//#define PAN_DERIVATIVE_GAIN       300	// 300 600
//#define TILT_PROPORTIONAL_GAIN    500	// 500 500
//#define TILT_DERIVATIVE_GAIN      300	// 400 700

#define PAN_PROPORTIONAL_GAIN     300	// 400 350
#define PAN_DERIVATIVE_GAIN       200	// 300 600
#define TILT_PROPORTIONAL_GAIN    350	// 500 500
#define TILT_DERIVATIVE_GAIN      300	// 400 700

#define SHAPE_X_MIN 0
#define SHAPE_X_MAX 222
#define SHAPE_Y_MIN 0
#define SHAPE_Y_MAX 252

#define PIXY_X_CENTER              ((SHAPE_X_MAX-SHAPE_X_MIN)/2)
#define PIXY_Y_CENTER              ((SHAPE_Y_MAX-SHAPE_Y_MIN)/2)

// These values will keep the tracked ball centered over the orange dot over the "i" in "rti" in the Shapes demo.
// Useful if you're tracking the orange ball.
//#define PIXY_X_CENTER              (168)
//#define PIXY_Y_CENTER              (89)  

class Gimbal {
public:
  Gimbal();

  void update_pan (DDS_Long x);
  void update_tilt (DDS_Long y);

  int32_t get_pan_position();
  int32_t get_tilt_position();

  struct GimbalParams {
    int32_t position;
    int32_t previous_error;
    int32_t proportional_gain;
    int32_t derivative_gain;
  } pan, tilt;

private:

  //-------------------------------------------------------------------
  // This calculates the position each axis of the cam control (pan/tilt)
  //-------------------------------------------------------------------
  void update(struct GimbalParams * gimbal, int32_t error);

};

} // namespace GIMBAL

#endif //GIMBAL_HPP
