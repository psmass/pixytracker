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

#ifndef REDUNDANCY_LAYER_HPP
#define REDUNDANCY_LAYER_HPP

#include <iostream>
#include <dds/dds.hpp>
#include "ddsEntities.hpp"
#include "led.hpp"

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

  
  #define TEN_SEC 10 // main loop clock tick is 1sec. 10 = ten sec
  #define ONE_SEC 1
  enum State {FAILED = 0, OK};
  enum Role {PRIMARY = 0, SECONDARY, TERTIARY, UNASSIGNED};
  enum SM_States {INITIALIZE, PREVOTE, VOTE, WAIT_VOTES_IN, VOTE_RESULTS, STEADY_STATE, \
		  SHUT_DOWN, ERROR};
  
  struct TrackerState {
    rti::core::Guid guid;
    int hbDeadlineCnt {0};
    enum State inconsistent_vote {FAILED}; // consistency issue with trackers vote
    enum State operational_hb {FAILED}; // indicates the trackers HB looks good
    Role role {UNASSIGNED};
    int votes[3] {0, 0, 0}; // votes for tracker role
    bool Ivoted {false}; // track if this tracker vote has been processed already

  }; 

  class RedundancyDb {  
  /* This class keeps all of the information as to Heartbeats received and 
       ordinal position etc. Needed by the Voting logic
  */
  public: 
    
    RedundancyDb(const dds::domain::DomainParticipant participant);
    ~RedundancyDb(void){}

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
    
    int getMyRoleStrength(void);
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
    bool iWasOperational(void) {return this->i_was_operational;}
    void setiWasOperational(bool wo_bool) {this->i_was_operational=wo_bool;}
    
    // Used to track 10 sec from last heartbeat in state INITIALIZE
    bool tenSecCount(void) {
      if (--this->ten_sec_count==0)
	return true;
      else
	return false;
    }

    void resetTenSecCount(void) { this->ten_sec_count = TEN_SEC; }

    void incVotesIn(void) { this->number_of_votes_in++;}
    int votesIn(void) {return this->number_of_votes_in;}
    void clearVotesIn(void) {this->number_of_votes_in = 1;} // our vote
    void clearIvoted(void) {
      for (int i=0; i<this->number_of_trackers; i++)
	this->array_tracker_states[i].Ivoted = false;
    }

    void clearVotes(void) { // clear votes and Ivoted for each tracker
      for (int i=0; i<3; i++) {
      	this->ordered_array_tracker_state_ptrs[i]->votes[0]=0;
      	this->ordered_array_tracker_state_ptrs[i]->votes[1]=0;
      	this->ordered_array_tracker_state_ptrs[i]->votes[2]=0;
        this->ordered_array_tracker_state_ptrs[i]->Ivoted=0;
      }
      this->number_of_votes_in = 1;  // init val - our own tracker
    }

    void setVotesExpected(int ve) {this->votes_expected = ve;}
    int votesExpected(void) {return this->votes_expected;}
    bool validateBallot(void);
    void printFullBallot(void);
    void printBallotVoteTracker(int t);
        
    void lostTracker(int tracker_ordinal);
    void assessVoteResults(void);
      
    TrackerState* getTrackerState_ptr(int i) {return ordered_array_tracker_state_ptrs[i];};
    // Each trackers own state is kept in array_tracker_state[0] 
    TrackerState* getMyTrackerStatePtr(void) { return &array_tracker_states[0]; }

    void printSortedTrackers(void);

    void printMyState(void);

    void updateLedStatus (LedControl* led_control);  // will be null() if no RPI
    
  private:
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

    // Indicates this tracker was previously operational (had been in steady
    // state vs. just coming on line.
    // Indicates this tracker is a late joiner (received a durable vote before
    // voting state) all trackers transition upon initialization to voting
    // together. 
    // In this case this tracker should silently join at next available role
    bool late_joiner {false};
    int votes_expected {3}; // changes number unique HBs + 1

    // track if we were previously operational - determines revoting
    bool i_was_operational {false};
    
    // The ordered_array_tracker_state_ptrs is always kept ordered
    // based on guid of each tracker (smallest to largest).
    // The ordinal is used to index into the ordered tracker array as an
    // optimization to get my tracker state info.
    // The ordinal, is also used to identify each tracker unit via LEDs
    // (especially when reporting other failed trackers). The ordinal is
    // adjusted as trackers come and go.
    // The ordinal is independent of role. Initially they will correlate
    // since the voting algorithm initially assigns role to tracker based
    // on ordinal. But once any tracker fails, a tracker may be promoted
    // and it's ordinal changed. A new tracker coming in to a running
    // system will not take over as primary even if it has the lowest guids
    // (Ordinal of 1). The existing Primary will remain Primary (avoiding
    // oscillation) independent of it's ordinal as the ordered array is
    // resorted
    int my_ordinal {1}; // ordinals of trackers are 1,2,3 and index the ordered * array
    TrackerState* ordered_array_tracker_state_ptrs[3]; 
  };

  
  class HeartbeatWtr : public Writer {
  public:
    HeartbeatWtr(
		 const dds::domain::DomainParticipant participant,
		 RedundancyDb* redundancy_db_obj,
		 bool periodic = false,
		 dds::core::Duration period=std::chrono::seconds(4));

    
    ~HeartbeatWtr(void){};

    // write() is effectively a runtime down cast for periodic data
    void write();
  
  private:
    RedundancyDb* redundancy_db_obj;
    
    
  };


  class HeartbeatRdr : public Reader {
  public:
    // Readers are not going to listen to thier own participants hbs
    HeartbeatRdr(const dds::domain::DomainParticipant participant,
		 RedundancyDb* redundancy_db_obj);
    
    ~HeartbeatRdr(void){};

    void handler(rti::sub::LoanedSample<rti::core::xtypes::DynamicDataImpl>* sample);

    private:
    RedundancyDb* redundancy_db_obj;
    
  };

  class VoteWtr : public Writer {
  public:
    VoteWtr(
	    const dds::domain::DomainParticipant participant,
	    RedundancyDb* redundancy_db_obj,
	    bool periodic = false,
	    dds::core::Duration period=std::chrono::seconds(4));

    ~VoteWtr(void) {};

    // runs voting algorithm and writes vote
    void vote(void);

    
    // write() is effectively a runtime down cast for periodic data
    void write() {};

  private:
    void setSampleField(std::string topic_field, rti::core::Guid guid);
    RedundancyDb* redundancy_db_obj;
    

  };

  class VoteRdr : public Reader {
  public:
    // Readers are not going to listen to thier own participants vote
    VoteRdr(const dds::domain::DomainParticipant participant,
	    RedundancyDb* redundancy_db_obj);   
    
    ~VoteRdr(void) {};

    void handler(rti::sub::LoanedSample<rti::core::xtypes::DynamicDataImpl>* sample);

  private:
    rti::core::Guid extractGuid(dds::core::xtypes::DynamicData& sample, std::string topicField);        RedundancyDb* redundancy_db_obj;

  };
} // namespace MODULE

#endif // REDUNDANCY_LAYER_HPP
