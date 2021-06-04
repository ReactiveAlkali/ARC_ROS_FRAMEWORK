/**
* CLASS: Task_center
* DATE: 07/27/20
* AUTHOR: Stuart Campbell
* DESCRIPTION: Centralizes most of the task based opperations
*/

#include <queue>
#include <vector>
#include <ros/ros.h>
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "arc_msgs/QuerryTask.h"
#include "arc_msgs/TaskService.h"
#ifndef ARC_TASKS_TASKCENTER_H
#define ARC_TASKS_TASKCENTER_H



namespace arc_tasks{

class Task_center {
  
 private:
  
  //The Task object that is used in the prioity queue
  struct Task{
    int task_type;
    int priority;
  };

  // The Compare function used by the priority queue
  // The higher the prioirty attribute the more
  // priority the object has
  struct task_compare{
    bool operator()(const Task& left, const Task& right){
      return left.priority > right.priority;
    }
  };

 
  std::priority_queue <class Task, std::vector<class Task>, task_compare > task_queue;
  
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle;

  //does exactly what it sounds like
  ros::Timer check_next_task_timer;
  
  //for checking the current task
  ros::ServiceServer curr_task_server;

  //for adding tasks to the queue
  ros::ServiceServer in_task_server;

  //==========================================
  //
  // The toggle clients for each task
  //
  //==========================================

  ros::ServiceClient toggle_explore_task_brain_client;

  Task curr_task;
  
  bool enabled;
  void toggle(bool state);
  void process();
  
 public:
  Task_center();
  bool toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
  void timer_cb(const ros::TimerEvent &event);
  void run();
  //void process();
  
  bool curr_task_cb(arc_msgs::QuerryTask::Request &req, arc_msgs::QuerryTask::Response &res);

  bool in_task_cb(arc_msgs::TaskService::Request &req, arc_msgs::TaskService::Response &res);

};
 
}
#endif //ARC_TASKS_TASKCENTER_H
