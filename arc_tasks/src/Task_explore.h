
#include "ros/ros.h"
#include <ros/ros.h>
#include "arc_msgs/Task.h"
#include "std_srvs/SetBool.h"

#ifndef ARC_TASKS_TASK_EXPLORE_H
#define ARC_TASKS_TASK_EXPLORE_H

namespace arc_tasks{
  class Task_explore{
  private:
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle;

    int DEFAULT_RANDOM_CHOICE_RATE = 10;

    //ping task current in task center
    ros::ServiceClient current_task;

    //send explore task to task center
    ros::ServiceClient send_task;

    //To toggle this specific task
    ros::ServiceServer toggle_server;

    //recieve command to interact with bot brain
    ros::ServiceServer toggle_brain_server;

    //The service to actually interact with the bot brain
    ros::ServiceClient send_toggle_list_brain_client;


    //toggle_explore_task_client: I dont think this does anything

    //timer to check on loop
    ros::Timer ping_task_timer;

    void toggle(bool state);
    void toggle_brain(bool state);

    bool enabled;
  public:
    void run();
    Task_explore();

    //void process_explore_task_cb();
    void timer_cb(const ros::TimerEvent &event);
    bool toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
    bool toggle_brain_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
  };

}
#endif //ARC_TASKS_TASK_EXPLORE_H
