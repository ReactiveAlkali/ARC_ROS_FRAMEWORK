#include "ros/ros.h"
#include <ros/ros.h>
#include "arc_msgs/Task.h"
#include "std_srvs/SetBool.h"


#ifndef ARC_TASKS_TASK_COMS_HANDLE_H
#define ARC_TASKS_TASK_COMS_HANDLE_H

namespace arc_tasks{
  class Task_manage_team{

  private:

    //====================================
    //
    // Boilerplate
    //
    //====================================

    bool enabled;

    ros::ServiceServer toggle_server;


    void toggle(bool state);
    
    //====================================
    //
    // Guided Debris Manage
    //
    //====================================

    ros::Subscriber guided_debris_sub;

    ros::Publisher debris_auction_pub;

    ros::Subscriber debris_auction_sub;

    ros::Publisher debris_confirm_pub;


    //====================================
    //
    // Confirm Victim
    //
    //====================================

    ros::Subscriber found_pot_vict_sub;

    ros::Publisher conf_vict_auction_pub;

    ros::Subscriber conf_vict_auction_sub;

    ros::Publisher conf_vict_confirm_pub;

  public:


    //==================================
    //
    // Basic functions
    //
    //==================================

    void run();

    bool toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

    
    Task_manage_team();


    //==================================
    //
    // Debris functions
    //
    //==================================

    bool process_debris_cb();

    bool process_debris_auction_cb();


    //===================================
    //
    // Confirm victim functions
    //
    //===================================

    
    bool process_found_pot_vict_cb();

    bool process_conf_vict_auction_cb();

    
    

  };

}





#endif //
