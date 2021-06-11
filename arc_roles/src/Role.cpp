
#include <std_srvs/SetBool.h>
#include <string>
#include "std_msgs/Bool.h"
#include "Role.h"
#include "arc_msgs/QuerryRole.h"
#define MAX_QUEUE_SIZE 1000
#define DEFAULT_RATE 10

using namespace arc_roles;

Role::Role(std::string local){

  this->priority = -1.0;
  this->max_count = -1;
  this->curr_count = 0;
    
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle(local);

  this->global_handle = global_handle;
  this->local_handle = local_handle;

  ROS_INFO("Setting up a role");

  this->declare_role = this->local_handle.advertiseService("declare_role",&Role::declare_role_cb,this);

    
  this->querry_role = this->local_handle.advertiseService("querry_role",&Role::querry_role_cb,this);

}

void Role::run(){
  ros::Rate r(DEFAULT_RATE);
  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
}

void Role::set_priority(float new_priority){
  this->priority = new_priority;
}

void Role::set_max_count(int new_max_count)
{
  this->max_count = new_max_count;
}

void Role::set_local_handle(std::string local_name){
  ROS_INFO("Setting a new Local Handle");
  ros::NodeHandle new_local_handle(local_name);
  this->local_handle = new_local_handle;

}


//==========================================
//
// Declaration of robots with this role
//
//==========================================

bool Role::declare_role_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res)
{
  this->handle_declare(req.data);
  return true;
}

void Role::handle_declare(bool in_indeclare)
{
  if (in_indeclare) {
    this->curr_count++;
  } else {
    this->curr_count--;
  }  
}

//==========================================
//
// Querry the suitability of the robot for this role
//
//==========================================

bool Role::querry_role_cb(arc_msgs::QuerryRole::Request &req, arc_msgs::QuerryRole::Response &res){
  res.app = this->handle_querry(req.bot_type);
  return true;
}

float Role::handle_querry(int in_bot_type){
  return 0.2;
}
