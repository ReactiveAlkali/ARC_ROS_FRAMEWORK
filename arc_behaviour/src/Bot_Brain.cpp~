#include <std_srvs/SetBool.h>
#include "std_msgs/Bool.h"
#include "Bot_Brain.h"
#include "arc_msgs/QuerryRole.h"
#include "arc_msgs/QuerryTask.h"
#include "arc_msgs/ToggleList.h"
#define MAX_QUEUE_SIZE 1000
#define DEFAULT_RATE 10

using namespace arc_behaviour;

Bot_Brain::Bot_Brain(){
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle("bot_brain");

  this->global_handle = global_handle;
  this->local_handle = local_handle;
  
  ROS_INFO("Setting up a bot brain");

  
  this->toggle_random_wander_client = global_handle.serviceClient<std_srvs::SetBool>("random_wander_ms/toggle");
  this->toggle_clean_debris_client = global_handle.serviceClient<std_srvs::SetBool>("clean_debris_ms/toggle");
  this->toggle_handle_marker_client = global_handle.serviceClient<std_srvs::SetBool>("handle_marker_ms/toggle");
  this->toggle_move_to_goal_client = global_handle.serviceClient<std_srvs::SetBool>("move_to_goal_ms/toggle");

  
  
  this->toggle_server = this->local_handle.advertiseService("toggle", &Bot_Brain::toggle_cb, this);

  this->in_toggle_list_server = this->local_handle.advertiseService("in_toggle_list", &Bot_Brain::in_toggle_list_cb, this);



//
//QUERRY FOR EFFICENCY
//

this->querry_general_client = global_handle.serviceClient<arc_msgs::QuerryRole>("/GENERAL/querry_role");
this->querry_lieutenent_client = global_handle.serviceClient<arc_msgs::QuerryRole>("/LIEUTENENT/querry_role");
this->querry_officer_client = global_handle.serviceClient<arc_msgs::QuerryRole>("/OFFICER/querry_role");

//
// DECLARE NEW ROLE
//

this->declare_general_client = local_handle.serviceClient<std_srvs::SetBool>("/GENERAL/declare_role");
this->declare_lieutenent_client = global_handle.serviceClient<std_srvs::SetBool>("/LIEUTENENT/declare_role");
this->declare_general_client = global_handle.serviceClient<std_srvs::SetBool>("/OFFICER/declare_role");




}



void Bot_Brain::run(){
  ros::Rate r(DEFAULT_RATE);

    while(ros::ok()){
      ros::spinOnce();
      r.sleep();
    }
}



bool Bot_Brain::toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    this->toggle(req.data);
    return true;
}

void Bot_Brain::toggle(bool state) {
    this->enabled = state;
    if(this->enabled) {
        ROS_INFO("Bot_Brain has been enabled.");
	this->setRole();
	
        //send an immediate goal right when we start.
    } else {
        ROS_INFO("Bot_Brain has been disabled.");
    }
}




bool Bot_Brain::in_toggle_list_cb(arc_msgs::ToggleList::Request &req, arc_msgs::ToggleList::Response &res){
  
  std_srvs::SetBool toggle_srv;

  ROS_INFO("IN BOT BRAIN SETTING UP BEHAVIOUR TOGGLE");

  
  //random wander toggle handle
  toggle_srv.request.data = req.random_wander;
  toggle_random_wander_client.call(toggle_srv);


  //move to goal
  toggle_srv.request.data = req.move_to_goal;
  toggle_move_to_goal_client.call(toggle_srv);

  
 
  return true;
}



//
//Currentlly very simple and does not adjust role ballence
//Usable only for new bots not bots with a role already.
//
void Bot_Brain::setRole(){

  //Inatialize all applicability values

  float general_app;
  float lieutenent_app;
  float officer_app;
  float debris_app;

  //
  //Querry all the avalable roles
  //
  
  arc_msgs::QuerryRole querry_role_srv;
  querry_role_srv.request.bot_type = 1;

  std_srvs::SetBool enable_srv;
  std_srvs::SetBool disable_srv;
  enable_srv.request.data = true;
  disable_srv.request.data = false;

  
  if(querry_general_client.call(querry_role_srv)){
    ROS_INFO("Querrying The General node");
    general_app = querry_role_srv.response.app;
    ROS_INFO("%f", general_app);

   } else{
    ROS_INFO("An Error occured querring the general node aborting role selection");
    return;
   }

  
   if(querry_lieutenent_client.call(querry_role_srv)){
    ROS_INFO("Querrying The Lieutenent node");
    lieutenent_app = querry_role_srv.response.app;
    ROS_INFO("%f", lieutenent_app);

   } else{
    ROS_INFO("An Error occured querring the Lieutenent node aborting role selection");
    return;
   }

   
    if(querry_officer_client.call(querry_role_srv)){
    ROS_INFO("Querrying The Officer node");
    officer_app = querry_role_srv.response.app;
    ROS_INFO("%f", officer_app);
   } else{
    ROS_INFO("An Error occured querring the officer node aborting role selection");
    return;
   }

    //
    //Best as a General
    //
    if(general_app >= lieutenent_app && general_app >= officer_app){

      //is currently a General
      /* if(this->bot_type ==1){
	return;
	//Do nothing because it is already doing its best
      }

      //is currently a lieutenent
      if(this->bot_type == 2){
	
	 if(declare_lieutenent_client.call(disable_srv)){
	   ROS_INFO("Undeclaring from the lieutenent node");
	 } else{
	   ROS_INFO("An Error occured undeclaring form lieutenent node aborting role selection");
	   return;
	 }

      }

      //is currently an officer
      if(this->bot_type == 3){
	 if(declare_officer_client.call(disable_srv)){
	   ROS_INFO("Undeclaring from the officer node");
	 } else{
	   ROS_INFO("An Error occured undeclaring form officer node aborting role selection");
	   return;
	 }

      }

      //is currently a debris
      if(this->bot_type == 4){

      }

      */

      this->setGeneralRole();
      this->bot_type = 1;
      return;
    }


    //
    //Best as a lieutenent
    //
     if(lieutenent_app >= general_app && lieutenent_app >= officer_app){

      //is currently a General
       /*
      if(this->bot_type ==1){
	 if(declare_general_client.call(disable_srv)){
	   ROS_INFO("Undeclaring from the general node");
	 } else{
	   ROS_INFO("An Error occured undeclaring from general node aborting role selection");
	   return;
	 }
	
      }

      //is currently a lieutenent
      if(this->bot_type == 2){
	return;
	//do nothing cause everything already G
      }

      //is currently an officer
      if(this->bot_type == 3){

      }

      //is currently a debris
      if(this->bot_type == 4){

      }
       */

      this->setLieutenentRole();
      this->bot_type = 2;
      return;
    }


     //
     //Best as an officer
     //
      if(officer_app >= lieutenent_app && officer_app >= general_app){

      //is currently a General
	/*
      if(this->bot_type ==1){
	
      }

      //is currently a lieutenent
      if(this->bot_type == 2){

      }

      //is currently an officer
      if(this->bot_type == 3){
	return;
	//do nothing as has best job in whole world

      }

      //is currently a debris
      if(this->bot_type == 4){

      }

	*/
      this->setOfficerRole();
      this->bot_type = 3;
      return;
    }

      //
      //Best as a Debris
      //

      /*
       if(general_app >= lieutenent_app && general_app >= officer_app){

      //is currently a General
      if(this->bot_type ==1){
	return;
	//Do
      }

      //is currently a lieutenent
      if(this->bot_type == 2){

      }

      //is currently an officer
      if(this->bot_type == 3){

      }

      //is currently a debris
      if(this->bot_type == 4){

      }

      this->setGeneralRole();
    }
      */ //TODO implement debris
  
  
}

void Bot_Brain::setGeneralRole(){
  std_srvs::SetBool enable_srv;
  std_srvs::SetBool disable_srv;
  enable_srv.request.data = true;
  disable_srv.request.data = false;
    if(toggle_random_wander_client.call(enable_srv)){
    ROS_INFO("General Role setting wander to true");
   } else{
    ROS_INFO("General Role error setting wander to true");
   }

    if(toggle_clean_debris_client.call(disable_srv)){
    ROS_INFO("General Role setting clean debris to false");
   } else{
    ROS_INFO("General Role error setting clean debris to false");
   }

     if(toggle_handle_marker_client.call(disable_srv)){
    ROS_INFO("General Role setting handle marker to false");
   } else{
    ROS_INFO("General Role error setting handle marker to false");
   }


    if(toggle_move_to_goal_client.call(enable_srv)){
    ROS_INFO("General Role setting move to goal to true");
   } else{
    ROS_INFO("General Role error setting move to goal to true");
   }

    
}

//TODO done
void Bot_Brain::setLieutenentRole(){
  std_srvs::SetBool enable_srv;
  std_srvs::SetBool disable_srv;
  enable_srv.request.data = true;
  disable_srv.request.data = false;
    if(toggle_random_wander_client.call(enable_srv)){
    ROS_INFO("Lieutenent Role setting wander to true");
   } else{
    ROS_INFO("Lieutenent Role error setting wander to true");
   }

    if(toggle_clean_debris_client.call(disable_srv)){
    ROS_INFO("Lieutenent Role setting clean debris to false");
   } else{
    ROS_INFO("Lieutenent Role error setting clean debris to false");
   }

     if(toggle_handle_marker_client.call(enable_srv)){
    ROS_INFO("Lieutenent Role setting handle marker to true");
   } else{
    ROS_INFO("Lieutenent Role error setting handle marker to true");
   }


    if(toggle_move_to_goal_client.call(enable_srv)){
    ROS_INFO("Lieutenent Role setting move to goal to true");
   } else{
    ROS_INFO("Lieutenent Role error setting move to goal to true");
   }

    
}


//TODO done
void Bot_Brain::setOfficerRole(){
  std_srvs::SetBool enable_srv;
  std_srvs::SetBool disable_srv;
  enable_srv.request.data = true;
  disable_srv.request.data = false;
    if(toggle_random_wander_client.call(enable_srv)){
    ROS_INFO("Officer Role setting wander to true");
   } else{
    ROS_INFO("Officer Role error setting wander to true");
   }

    if(toggle_clean_debris_client.call(disable_srv)){
    ROS_INFO("Officer Role setting clean debris to false");
   } else{
    ROS_INFO("Officer Role error setting clean debris to false");
   }

     if(toggle_handle_marker_client.call(disable_srv)){
    ROS_INFO("Officer Role setting handle marker to false");
   } else{
    ROS_INFO("Officer Role error setting handle marker to false");
   }


    if(toggle_move_to_goal_client.call(disable_srv)){
    ROS_INFO("Officer Role setting move to goal to false");
   } else{
    ROS_INFO("Officer Role error setting move to goal to false");
   }

    
}
