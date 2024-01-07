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

#ifndef TOPICS_HPP
#define TOPICS_HPP

#include <iostream>
#include <dds/dds.hpp>
#include "ddsEntities.hpp"
#include "gimbal.hpp"


namespace MODULE
{

    /* How to use specific topic Readers and Writers:

    The Topic specific Reader Constructor -  can be used to update Topic Specific Content filters 
     - e.g. in the case of a device Reader, it should register for myTopic - i.e.
     commands directed specifically to it from a controller.

    The controller readers likely would not want to filter for a specific deviceID,
    as generally controllers handle all the data sent to them. Controllers typically
    are not a specific target ID either.

    The Topic specific Writer member functions. 

    Topic Writer have the topic specific 
    Specific Writer Handlers have two parts, the Initial Setup and the Handler Loop.
    The user may want add code  in the Initial Setup (prior to the Handler Loop) to
    statically set the source ID (in the case of a device) or any other static data. 
    This is done once and is topic specific.

    Code can be added in the loop write the topic periodically. If non periodic, the loop
    can be used for writer event status and sleep periodically with no write operation.
    A separate writeData(data) member funcstion can be added to the specific topic class to
    allow the main program to set data and write at will.

*/
  
  enum State {FAILED = 0, OPERATIONAL};
  enum Roll {PRIMARY = 0, SECONDARY, TERTIARY, UNASSIGNED};
  
  struct TrackerState {
    rti::core::Guid guid;
    bool Ivoted {false}; // track if this tracker vote has been processed already
    int votes[3] {0, 0, 0}; // votes for tracker w/Guid {Primary, Secondary, Tertiary}
    int hbDeadlineCnt[3] {0, 0, 0};
    enum State state {FAILED};
    Roll roll {UNASSIGNED};
  }; 

  class RedundancyInfo {  
  /* This class keeps all of the information as to Heartbeats received and 
       ordinal position etc. Needed by the Voting logic
  */
  public: 
    
    RedundancyInfo(const dds::domain::DomainParticipant participant);
    ~RedundancyInfo(void){}

    int getMyOrdinal(void) {return this->my_ordinal;};
    rti::core::Guid getMyGuid() {
      return this->ordered_array_tracker_state_ptrs[this->my_ordinal-1]->guid;}
    int getMyRollStrength(void);
    void sortSaveHbGuid(rti::core::Guid hb_guid);
    int numberOfTrackers(void) {return this->number_of_trackers;}
    void incVotesIn(void) {this->number_of_votes_in++;}
    int votesIn(void) {return this->number_of_votes_in;}
    void assessVoteResults(void);

    void clearVotes(void) { // clear votes and Ivoted for each tracker
    for (int i=0; i<this->number_of_trackers; i++) {
	this->ordered_array_tracker_state_ptrs[i]->votes[0]=0;
	this->ordered_array_tracker_state_ptrs[i]->votes[1]=0;
	this->ordered_array_tracker_state_ptrs[i]->votes[2]=0;
        this->ordered_array_tracker_state_ptrs[i]->Ivoted=0;
      }
    }

    void clearTrackerData(int tracker); // nulls out the Guid & roll, marks tracker FAILED
      
    TrackerState* getTrackerState_ptr(int i) {return ordered_array_tracker_state_ptrs[i];};
    // Each trackers own state is kept in array_tracker_state[0] 
    TrackerState* getMyTrackerStatePtr(void) { return &array_tracker_states[0]; }

    void clearIvoted(void) {
      for (int i=0; i<this->number_of_trackers; i++)
	this->array_tracker_states[i].Ivoted = false;
    };

    void printVoteResults() {
      for (int i=0; i<this->numberOfTrackers(); i++)
	std::cout << "\nFor Tracker: "
		  << this->ordered_array_tracker_state_ptrs[i]->guid
		  << "\nVotes for Primary: "
		  << this->ordered_array_tracker_state_ptrs[i]->votes[0]
		  << "\nVotes for Secondary: "
		  << this->ordered_array_tracker_state_ptrs[i]->votes[1]
		  << "\nVotes for Tertiary: "
		  << this->ordered_array_tracker_state_ptrs[i]->votes[2]
		  << std::endl;
    };	  	  
    
  private:
    int my_ordinal {1}; // ordinals of trackers are 1,2,3 and index the ordered * array
    int number_of_trackers {1};
    int number_of_votes_in {1}; // 1 is our own internal vote
    rti::core::Guid primary, secondary, tertiary;
    TrackerState array_tracker_states[3];
    TrackerState* ordered_array_tracker_state_ptrs[3];
  };
    
  class ServoWtr : public Writer {
  public:
    ServoWtr(
	     const dds::domain::DomainParticipant participant,
	     bool periodic = false,
	     dds::core::Duration period =std::chrono::seconds(4));
    ~ServoWtr(void){};

    // allows us to enable and disable writing (see writeData() member func)
    void enable(void) {this->enabled_to_write = true; };
    void disable(void) {this->enabled_to_write = false; };
    void printGimbalPosition(void) {
	std::cout << "P: " << gimbal.get_pan_position()			\
	       	  <<" T: " << gimbal.get_tilt_position()		\
		  << "          ""\r" << std::flush;
    };


    // write() is effectively a runtime down cast for periodic data
    // void write();
  
    // Servo Specific non-periodic write call
    void writeData(int32_t x, int32_t y);
        
  private:
    DDS_DynamicData * servo_data;
    GIMBAL::
    Gimbal gimbal;
    int frame_count {0};
    bool enabled_to_write {false};
  };

  class ShapesRdr : public Reader {
  public:
    ShapesRdr(const dds::domain::DomainParticipant participant,
	      ServoWtr* servoWriter);
    ~ShapesRdr(void){};

    void handler(dds::core::xtypes::DynamicData& data);
        
  private:
    DDS_Long x;
    DDS_Long y;

    ServoWtr* servo_writer;

  };

  class HeartbeatWtr : public Writer {
  public:
    HeartbeatWtr(
		 const dds::domain::DomainParticipant participant,
		 RedundancyInfo* redundancy_info_obj,
		 bool periodic = false,
		 dds::core::Duration period=std::chrono::seconds(4));

    
    ~HeartbeatWtr(void){};

    // write() is effectively a runtime down cast for periodic data
    void write();
  
  private:
    RedundancyInfo* my_redundancy_info_obj;
    
    
  };


  class HeartbeatRdr : public Reader {
  public:
    // Readers are not going to listen to thier own participants hbs
    HeartbeatRdr(const dds::domain::DomainParticipant participant,
		 RedundancyInfo* redundancy_info_obj);
    
    ~HeartbeatRdr(void){};

    void handler(dds::core::xtypes::DynamicData& data);

    private:
    RedundancyInfo* my_redundancy_info_obj;
    
  };

  class VoteWtr : public Writer {
  public:
    VoteWtr(
	    const dds::domain::DomainParticipant participant,
	    RedundancyInfo* redundancy_info_obj,
	    bool periodic = false,
	    dds::core::Duration period=std::chrono::seconds(4));

    ~VoteWtr(void) {};

    // runs voting algorithm and writes vote
    void vote(void);

    
    // write() is effectively a runtime down cast for periodic data
    void write() {};

  private:
    void setSampleField(std::string topic_field, rti::core::Guid guid);
    RedundancyInfo* my_redundancy_info_obj;
    

  };

  class VoteRdr : public Reader {
  public:
    // Readers are not going to listen to thier own participants vote
    VoteRdr(const dds::domain::DomainParticipant participant,
	    RedundancyInfo* redundancy_info_obj);   
    
    ~VoteRdr(void) {};

    void handler(dds::core::xtypes::DynamicData& data);

  private:
    rti::core::Guid extractGuid(dds::core::xtypes::DynamicData& sample, std::string topicField);        RedundancyInfo* my_redundancy_info_obj;


  };
} // namespace MODULE

#endif // TOPICS_HPP
