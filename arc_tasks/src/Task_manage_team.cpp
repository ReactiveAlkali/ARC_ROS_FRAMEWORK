#include "Task_manage_team.h"

#define ROS_RATE 10 //hz
#define MAX_QUEUE_SIZE 1000 //max number of messages to keep on queue before flushing


using namespace arc_tasks;


Task_manage_team::Task_manage_team(){
  
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle("task_manage_team"):

  this->global_handle = global_handle;
  this->local_handle = local_handle;

  
  ROS_INFO("Setting up the mannage team task");

  //==============================================
  //
  //  Guided Debris Manage
  //
  //==============================================

  //TODO
  this->guided_debris_sub = global_handle.subscribe("debris_request", MAX_QUEUE_SIZE, &Task_manage_team::process_debris_cb,this);
  
  //TODO
  this->debris_auction_pub = global_handle.advertise<arc_msgs::Debris>("debris_auction", MAX_QUEUE_SIZE);

  //TODO
  this->debris_auction_sub = global_handle.subscribe("debris_auction", MAX_QUEUE_SIZE, &Task_manage_team::process_debris_auction_cb, this);

  //TODO
  this->debris_confirm_pub = global_handle.advertise<arc_msgs::Auction_bid>("confirm_debris_auction", MAX_QUEUE_SIZE);


  //==============================================
  //
  //  Confirm Victim
  //
  //==============================================

    //TODO
  this->found_pot_vict_sub = global_handle.subscribe("", MAX_QUEUE_SIZE, &Task_manage_team::process_found_pot_vict_cb,this);
  
  //TODO
  this->conf_vict_auction_pub = local_handle.advertise<arc_msgs::DetectedVictim>("conf_vict_auction", MAX_QUEUE_SIZE);

  //TODO
  this->conf_vict_auction_sub = global_handle.subscribe("conf_vict_auction", MAX_QUEUE_SIZE, &Task_manage_team::process_conf_vict_auction_cb, this);

  //TODO
  this->conf_vict_confirm_pub = local_handle.advertise<arc_msgs::Auction_bid>("confirm_conf_vict_auction", MAX_QUEUE_SIZE);



  this->enabled = false;


}



//===============================
//
// Boilerplate functions
//
//===============================

void Task_manage_team::run(){
    ros::Rate r(ROS_RATE);

    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}



bool Task_manage_team::toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    this->toggle(req.data);
    return true;
}

void Task_manage_team::toggle(bool state) {
    this->enabled = state;

    if(this->enabled) {
        ROS_INFO("CleanDebrisMS has been enabled.");
    } else {
        ROS_INFO("CleanDebrisMS has been disabled.");
    }
}


//=================================
//
// Debris Callback Functions
//
//=================================




bool Task_manage_team::process_debris_cb(){
  if(this->enabled){
    
  }
}



bool Task_manage_team::process_debris_auction_cb(){
  if(this->enabled){

  }
}



//====================================
//
//Confirm Victim Functions
//
//====================================


bool Task_manage_team::process_found_pot_vict_cb(){
  if(this->enabled){
    
  }
}

bool Task_manage_team::process_cong_vict_auction_cb(){
  if(this->enabled){

  }
}


