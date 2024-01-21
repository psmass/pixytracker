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

/* led class provides access via system command raspi-gpio commands to drive the LEDs
 * on the custom Tracker LED pi-hat. 
 *
 * LEDs function and name: 
 * 
 *
 *                      |        ROLE                     |  STATUS |     
 *			+——————————+———————————+——————————+—————-———+
 *                      |   LED1   |   LED2    |   LED3   |  LED4   |
 *			+——————————+———————————+——————————+—————————+
 *			| Primary  | Secondary | Tertiary | OK/Fail |
 *			+——————————+———————————+——————————+—————————+
 *
 *  RPI GPIO HEADER Pins LEDs as follows
 *
 *   LED     GPIO pins    GREEN     RED      OFF (alt OFF)
 *  =======================================================
 *  LED1     20 & 21     20H-21L  20L-21H  20L-21L(20H-21H)
 *  LED2     7  & 8       7H-8L    7L-8H    7L-8L ( 7H-8H ) 
 *  LED3     23 & 24     23L-24H  23H-24L  23L-24L(23H-24H)
 *  LED4     14 & 15     14L-15H  14H-15L  14L-15L(14H-15H)  
 */

// #define LED_HPP // comment out for Raspberry to activate LEDs

#ifndef LED_HPP
#define LED_HPP

#include <tuple>
#include <sstream>
#include <string>
#include <iostream>
#include <unistd.h> // send linux command
#include <stdlib.h>
#include "topics.hpp"


namespace MODULE
{


  class LedControl {

  public:
    LedControl(void) {
      std::tuple<int, int> led_pins;
      // set all led pins as out put "raspi-gpio set <pin> op"
      for (int i=0; i<4; i++) {
	led_pins=this->leds_pin_vec[i];
	this->sendCommand (std::get<0>(led_pins), "op");
	this->sendCommand (std::get<1>(led_pins), "op");
	this->setLedOff(i); // and ensure it's off
      }
    };
    
    ~LedControl(void) {};

    void setLedGreen(int led) {
      this->sendCommand (std::get<0>(this->leds_pin_vec[led]), "dl");
      this->sendCommand (std::get<1>(this->leds_pin_vec[led]), "dh");
    };
    
    void setLedRed(int led) {
      this->sendCommand (std::get<0>(this->leds_pin_vec[led]), "dh");
      this->sendCommand (std::get<1>(this->leds_pin_vec[led]), "dl");
    };
    
    void setLedOff(int led) {
      this->sendCommand (std::get<0>(this->leds_pin_vec[led]), "dl");
      this->sendCommand (std::get<1>(this->leds_pin_vec[led]), "dl");
    };

    void setLedRole (enum Role role) {
      for (int i=1; i<3; i++)
	setLedOff(i); // turn off all LEDs
      
      switch (role) { // turn on the correct role
      case PRIMARY:
	this->setLedGreen(1);
	break;
      case SECONDARY:
     	this->setLedGreen(2);
	break;
      case TERTIARY:
        this->setLedGreen(3);
	break;
      default:
	break;
	}
     
    };

    
  private:
      void sendCommand (int pin, std::string command) {
	std::stringstream raspi_gpio_cmd;
	raspi_gpio_cmd << "raspi-gpio set "
		       << pin
		       << " "
		       << command
		       << std::flush;
	//system (raspi_gpio_cmd);
      };
	
    // led_pin_vec[vector_idx]<pin> e.g. leds_pin_vec[2]<0> returns 23
    std::vector<std::tuple<int, int>> leds_pin_vec {  // LEDs 1-4, 
      std::tuple<int, int> { 21, 20 }, // Green pin Low or Red pin high first
      std::tuple<int, int> {  8, 7 },
      std::tuple<int, int> { 23, 24 },
      std::tuple<int, int> { 14, 15 }	  
    };
  };


} // namespace MODULE
#endif
       
