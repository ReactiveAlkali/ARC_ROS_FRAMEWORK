#include <std_srvs/Trigger.h>
#include "Task_explore.h"
#include "arc_msgs/TaskService.h"
#include "arc_msgs/ToggleList.h"
#include "arc_msgs/QuerryTask.h"

using namespace arc_tasks;

Task_explore::Task_explore(){
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle("task_explore");

  this->global_handle = global_handle;
  this->local_handle = local_handle;
  ROS_INFO("Setting up the explore task for a bot");


  
  //Turns this task on and off
  this->toggle_server = this->local_handle.advertiseService("toggle", &Task_explore::toggle_cb, this);

  //querries the task center for the current task
  this->current_task = this->global_handle.serviceClient<arc_msgs::QuerryTask>("task_center/curr_task");

  //sends an explore task to the task center
  this->send_task = this->global_handle.serviceClient<arc_msgs::TaskService>("task_center/in_task");


  //recieves the command to interact with the bot brain from task center
  this->toggle_brain_server = this->local_handle.advertiseService("toggle_brain",&Task_explore::toggle_brain_cb, this);

  
  
  //sends a list of behaviours to the bot brains to begin the explore task
  this->send_toggle_list_brain_client = this->global_handle.serviceClient<arc_msgs::ToggleList>("bot_brain/in_toggle_list");

  
  ros::Timer timer = global_handle.createTimer(ros::Duration(20), &Task_explore::timer_cb, this, false);
  this->ping_task_timer = timer;



  
  this->toggle(true);

  

  //starts the timer
  this->ping_task_timer.start();
  ROS_INFO("If you reached this The explore task is done");

}

//==================================
//
//  Turns this task on and off
//
//==================================


bool Task_explore::toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    this->toggle(req.data);
    return true;
}

void Task_explore::toggle(bool state) {
    this->enabled = state;
    if(this->enabled) {
        ROS_INFO("Explore task has been enabled.");

    } else {
        ROS_INFO("Explore task has been disabled.");
    }
}


//==================================
//
//  function gets called on a timer
//
//==================================

void Task_explore::timer_cb(const ros::TimerEvent &event) {
    if(this->enabled) {
        arc_msgs::QuerryTask req;
	//============================
	//
	//       Needs Odom
	//
	//============================
	arc_msgs::TaskService expl_task;
        this->current_task.call(req);
	sleep(2);
	if(req.response.task_type == 0){
	  ROS_INFO("TASK EXPLORE WAS PASSED TASK 0");
	  expl_task.request.task_type = 1;
	  expl_task.request.priority = 1;
	  this->send_task.call(expl_task);

	}
        ROS_INFO("Timer Called explore task");
    }
}


  //============================
  //
  //  sends a msg off the the bot brain
  //  to activate the behaviours
  //
  //============================

bool Task_explore::toggle_brain_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res){


  ROS_INFO("PLEASE WORK");
  this->toggle_brain(req.data);
  
  return true;
}

void Task_explore::toggle_brain(bool state){
  arc_msgs::ToggleList toggle;

   ROS_INFO("IN EXPLORE DOING TOGGLE SETUP");
  if(state){

    ROS_INFO("ENABLING EXPLORE TASK");
    toggle.request.random_wander = true;
    toggle.request.move_to_goal = false;
    toggle.request.clean_debris = false;
    toggle.request.detect_debris = false;

    this->send_toggle_list_brain_client.call(toggle);
    ROS_INFO("SENT THE TOGGLE MSG");
  }
  else{
    ROS_INFO("This has not beed implemented yet");
      return;
  }

  this->send_toggle_list_brain_client.call(toggle);
}


//=======================================
//
//  The Main run loop, false will kill
//
//=======================================


void Task_explore::run() {
    ros::Rate r(10);
    //set a timer to call random generation of goals every once in a while

    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

}
