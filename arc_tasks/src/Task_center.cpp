#include <std_srvs/SetBool.h>
#include "std_msgs/Bool.h"
#include "Task_center.h"
#include "arc_msgs/QuerryTask.h"
#include "arc_msgs/TaskService.h"
#include <ros/ros.h>


#define MAX_QUEUE_SIZE 1000
#define DEFAULT_RATE 10
#define TIMER_PERIOD 10



// Task Center
// Function: The task center is the node that handles the priority queue of tasks and dequeues the next task and processess it.
// Tasks in the queue are defined as a struct, which itself is defined in the header file.
// Esseciencally, The Task struct has an int task_type to define
// which task is being performed, a table of tasks and their id is proveded in the arc_ros directory.
// The Tasks also have an int priority which is used to
// enqueue them in to the priority queue.

// Author: Stuart Campbell
// stuartcharles@outlook.com for questions


using namespace arc_tasks;

Task_center::Task_center(){

  //boilerplate
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle("task_center");


  //The task queue should start as "running"
  this->toggle(true);

  this->global_handle = global_handle;
  this->local_handle = local_handle;
  ROS_INFO("SETTING UP A TASK CENTER");



  //=======================================
  //
  // Setting up the queue and current task
  //
  //=======================================

  //A task type 0 means to Idle. In tests this method works to start up the queue
  //TODO test if the queue performs as expected without the beginer task
  this->curr_task = {.task_type = 0, .priority = 0};


  //Creates a timer that checks the top Task, and calls the timer_cb function
  ros::Timer timer = global_handle.createTimer(ros::Duration(TIMER_PERIOD), &Task_center::timer_cb, this, false);
  this->check_next_task_timer = timer;

  //Starts the timer
  this->check_next_task_timer.start();

  //======================================
  //
  //  Setting up some service servers
  //
  //======================================

  //service to give a msg responce the current task
  this->curr_task_server = this->local_handle.advertiseService("curr_task", &Task_center::curr_task_cb,this);

  //service to take task msgs and queue them into the priority queue
  this->in_task_server = this->local_handle.advertiseService("in_task", &Task_center::in_task_cb,this);


  //=======================================
  //
  // Setting up task call backs
  //
  //=======================================

  //setting up the service to toggle this specific node
  this->toggle_explore_task_brain_client = global_handle.serviceClient<std_srvs::SetBool>("task_explore/toggle_brain");


  ROS_INFO("Finished setting up task center");

}


//the toggle callback function
bool Task_center::toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res){
    this->toggle(req.data);
    return true;
}


//
//
//
//
//

void Task_center::toggle(bool state){
  this->enabled = state;
    if(this->enabled){
      ROS_INFO("Task Center has been enabled");

      //starts the timer if stopped
      this->check_next_task_timer.start();
    }
    else{
      ROS_INFO("Task Center has been disabled");

      //stops the timer if started
      this->check_next_task_timer.stop();

      //resets the queue to empty
      //TODO test functionality
      this->task_queue = std::priority_queue <class Task, std::vector<class Task>, task_compare > ();
    }
}


//==========================================================
//
// Looks at the queue and initiates or keeps the tasks
//
//==========================================================

void Task_center::process(){
  ROS_INFO("Doing some processing stuff");
  if(enabled){
    if(!task_queue.empty() && task_queue.top().priority > curr_task.priority){
      ROS_INFO("Task In queue is better than current task so switching");
      this->curr_task = task_queue.top();
       task_queue.pop();

       //==================================
       //
       // Dealing with each task call back
       //
       //==================================

       std_srvs::SetBool enable_srv;
       enable_srv.request.data = true;

       //explore task
       if(curr_task.task_type == 1){
	 ROS_INFO("TELLING EXPLORE TASK TO TAlK TO BRAINS");
	 this->toggle_explore_task_brain_client.call(enable_srv);
	 ROS_INFO("PASSED THE HARD PART");
       }

       //unguided debris task

       //guided debris task

       //confirm victim task

       //Manage Team task

       
    }

  }
}


//The timer call back
void Task_center::timer_cb(const ros::TimerEvent &event) {
    if(this->enabled) {

      this->process();
      ROS_INFO("Timer looked at queue");
    }
}




//Is giving the task center node a task to add to the queue
bool Task_center::in_task_cb(arc_msgs::TaskService::Request &req, arc_msgs::TaskService::Response &res){

  Task build_task = {.task_type = req.task_type, .priority = req.priority};
  task_queue.push(build_task);
  ROS_INFO("Pushing a task onto the queue, Will be Explore task cause thats all there is");
  return true;
}



//wants you to give the msg the details of the current task
bool Task_center::curr_task_cb(arc_msgs::QuerryTask::Request &req, arc_msgs::QuerryTask::Response &res){
  res.task_type = this->curr_task.task_type;
  res.priority = this->curr_task.priority;
  return true;
}










//=======================================
//
//  The Main run loop, false will kill
//
//=======================================


void Task_center::run() {
    ros::Rate r(DEFAULT_RATE);

    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

}
