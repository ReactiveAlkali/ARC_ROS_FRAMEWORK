/**
* CLASS: TaskCenter
* DATE: 07/27/20
* AUTHOR: Stuart Campbell
* DESCRIPTION: Task Core
*/

#include <queue>
#include <ros/ros.h>
#ifndef ARC_TASKS_TASKCENTER_H
#define ARC_TASKS_TASKCENTER_H

class TaskCenter {
  
 private:
  
  //priority_queue <Task>
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle;

  //client to send for toggling random wandering
  ros::ServiceClient toggle_random_wander_client;
  
  bool active;
  
 public:
  TaskCenter();
  bool toggle();
  bool process();

};

#endif //ARC_TASKS_TASKCENTER_H
