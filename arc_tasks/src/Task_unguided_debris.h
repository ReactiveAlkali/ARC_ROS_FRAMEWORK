#include "ros/ros.h"
#include <ros/ros.h>
#include "arc_msgs/Task.h"
#include "std_srvs/SetBool.h"


#ifndef ARC_TASKS_TASK_UNGUIDED_DEBRIS_H
#define ARC_TASKS_TASK_UNGUIDED_DEBRIS_H

namespace arc_tasks{
  class Task_unguided_debris{
  private:
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle;

    //ping task current in task center
    ros::ServiceClient current_task;

    //send unguided_debris to task center
    ros::ServiceClient send_task;

    //to toggle this specific task
    ros::ServiceServer toggle_server;

    //recieve command to interact with bot brain
    ros::ServiceServer toggle_brain_server;

    //the service to actually interact with the bot brain
    ros::ServiceClient send_toggle_list_brain_client;



    //timer to check on a loop
    ros::Timer ping_task_timer;


    void toggle(bool state);
    void toggle_brain(bool state);

    bool enabled;
  public:
    void run();
    Task_unguided_debris();

    void process_unguided_debris_task_cb();
    void timer_cb(const ros::TimerEvent &event);

    bool toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolRequest &res);
    bool toggle_brain_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

  };

}

#endif //ARC_TASKS_TASK_UNGUIDED_DEBRIS_H
