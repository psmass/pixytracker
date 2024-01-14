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

  
  #define TEN_SEC 40 // main loop clock tick is 250ms. 40 = ten sec
  #define ONE_SEC 4
  enum State {FAILED = 0, OPERATIONAL};
  enum Roll {PRIMARY = 0, SECONDARY, TERTIARY, UNASSIGNED};
  enum SM_States {INITIALIZE, POSTINIT, VOTE, WAIT_VOTES_IN, VOTE_RESULTS, STEADY_STATE, \
		  SHUT_DOWN, ERROR};
  
  struct TrackerState {
    rti::core::Guid guid;
    bool Ivoted {false}; // track if this tracker vote has been processed already
    int votes[3] {0, 0, 0}; // votes for tracker w/Guid {Primary, Secondary, Tertiary}
    int hbDeadlineCnt {0};
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
    bool validateMyOrdinal(void) {
      bool verified {false};
      for (int i=0; i<this->number_of_trackers; i++) {
	if ((this->ordered_array_tracker_state_ptrs[i]->guid ==
	     this->my_guid) && (i+1 == this->my_ordinal)) {
	  verified = true;
	  break;
	}
      }
      return verified;
    }

    enum SM_States smState(){return this->sm_state;}
    void setSM_State (enum SM_States my_state) {this->sm_state = my_state;}
	   
    rti::core::Guid getMyGuid(void) {return this->my_guid;}
    void setMyGuid(rti::core::Guid guid) {this->my_guid = guid;}
    rti::core::Guid getNullGuid(void) { return this->ff_guid; }
    
    int getMyRollStrength(void);
    void sortSaveGuids(void);
    
    int numberOfTrackers(void) {return this->number_of_trackers;}
    // inc Trackers only by receiving new Heartbeats to avoid CERR
    void incNumberOfTrackers(void) {
      if (this->number_of_trackers == 3)
	std::cerr << "ERROR: Attempted inc to 4 Trackers" << std::endl;
      else this->number_of_trackers++;
    }

    bool isNewTracker(void) {return this->is_new_tracker;}
    void setNewTracker(bool nt_bool) {this->is_new_tracker=nt_bool;}
    bool isLateJoiner(void) {return this->late_joiner;}
    void setLateJoiner(bool lj_bool) {this->late_joiner=lj_bool;}
    
    // Used to track 10 sec from last heartbeat in state INITIALIZE
    bool tenSecCount(void) {
      if (--this->ten_sec_count==0)
	return true;
      else
	return false;
    };

    void resetTenSecCount(void) { this->ten_sec_count = TEN_SEC; }

    void incVotesIn(void) {this->number_of_votes_in++;}
    void setVotesIn(int votes) { this->number_of_votes_in = votes;}
    int votesIn(void) {return this->number_of_votes_in;}
    void clearVotesIn(void) {this->number_of_votes_in = 1;} // our vote
    void setVotesExpected(int ve) {this->votes_expected = ve;}
    int votesExpected(void) {return this->votes_expected;}
    
    void lostTracker(int tracker_ordinal);
    void assessVoteResults(void);

    void clearVotes(void) { // clear votes and Ivoted for each tracker
    for (int i=0; i<this->number_of_trackers; i++) {
      this->clearVotesTracker(i);
      }
    }
      
    TrackerState* getTrackerState_ptr(int i) {return ordered_array_tracker_state_ptrs[i];};
    // Each trackers own state is kept in array_tracker_state[0] 
    TrackerState* getMyTrackerStatePtr(void) { return &array_tracker_states[0]; }

    void clearIvoted(void) {
      for (int i=0; i<this->number_of_trackers; i++)
	this->array_tracker_states[i].Ivoted = false;
    };

    void printVoteResults(void) {
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
    }
    
    void printSortedTrackers(void);

    void printMyState(void) {
      // This is the equivalent of what the 4 LEDs on a Rpi will provide.
      // Forth LED (l to r) -  if Green indicates this tracker is Primary
      //
      // First (1), Second (2), and Third (3) LEDs indicates this Trackers
      // Ordinal (Green) and operational status {Failed(Red), Operational(Off)}
      // of the corresponding redundant tracker.
      // If one of these Status LEDs is off the corresponding tracker with the
      // same Ordinal should be Green. Any
      // of these LEDs that is Red the corresponding tracker will be off and
      // can be deduced from the other redundant trackers Green Ordinal LED.
      //
      std::cout << "|  ORDINAL1  |  ORDINAL2  |  ORDINAL3  | PRIMARY |" << std::endl;
      std::cout << "+------------+------------+------------+---------+" << std::endl;
      std::cout << "|";
      
      for (int i=0; i<3; i++) {
	if (this->ordered_array_tracker_state_ptrs[i]->state == FAILED) {
	  std::cout << "FAILED (Red)|";
	}
	if (this->ordered_array_tracker_state_ptrs[i]->state == OPERATIONAL) {
	  if (i+1 == this->my_ordinal) 
	    std::cout << "  OK (Grn)  |";
          else 
            std::cout << "            |";
        }
      } // for
      if (this->array_tracker_states[0].roll == PRIMARY)
        std::cout << "  GREEN  |" << std::endl;
      else
	 std::cout << "         |" << std::endl;
  
      std::cout << "+------------+------------+------------+---------+" << std::endl;
    }
      
      
  private:
    void clearVotesTracker(int tracker_indx) {
	this->ordered_array_tracker_state_ptrs[tracker_indx]->votes[0]=0;
	this->ordered_array_tracker_state_ptrs[tracker_indx]->votes[1]=0;
	this->ordered_array_tracker_state_ptrs[tracker_indx]->votes[2]=0;
        this->ordered_array_tracker_state_ptrs[tracker_indx]->Ivoted=0;
	this->number_of_votes_in = 1;  // init val - our own tracker
    }

    enum SM_States sm_state {INITIALIZE}; // keep this trackers State Machine State
    int ten_sec_count {TEN_SEC}; // ten sec count based on main loop period

    rti::core::Guid my_guid; // Save my own guid separately to validate ordinal
                             // my_guid = HB Writer instance handle - see HB wtr C'tor
    int number_of_trackers {1};
    int number_of_votes_in {1}; // 1 is our own internal vote
    rti::core::Guid ff_guid; // handy for future use
    rti::core::Guid primary, secondary, tertiary;
    TrackerState array_tracker_states[3];

    // Used to ensure voting is correctly filled out. Set true in HB Reader if
    // new tracker is detected. Set false in loss of a tracker.
    bool is_new_tracker {true};

    // indicates this tracker is a late joiner and should not vote
    // and should silently join at next available roll
    bool late_joiner {false};
    int votes_expected {3}; // changes by late joiners 
    
    // The ordered_array_tracker_state_ptrs is always kept ordered
    // based on guid of each tracker (smallest to largest).
    // The ordinal is used to index into the ordered tracker array as an
    // optimization to get my tracker state info.
    // The ordinal, is also used to identify each tracker unit via LEDs
    // (especially when reporting other failed trackers). The ordinal is
    // adjusted as trackers come and go.
    // The ordinal is independent of roll. Initially they will correlate
    // since the voting algorithm initially assigns roll to tracker based
    // on ordinal. But once any tracker fails, a tracker may be promoted
    // and it's ordinal changed. A new tracker coming in to a running
    // system will not take over as primary even if it has the lowest guids
    // (Ordinal of 1). The existing Primary will remain Primary (avoiding
    // oscillation) independent of it's ordinal as the ordered array is
    // resorted
    int my_ordinal {1}; // ordinals of trackers are 1,2,3 and index the ordered * array
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
