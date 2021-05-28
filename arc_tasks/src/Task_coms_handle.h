#include "ros/ros.h"
#include <ros/ros.h>
#include "arc_msgs/Task.h"
#include "std_srvs/SetBool.h"

#ifndef ARC_TASKS_TASK_COMS_HANDLE_H
#define ARC_TASKS_TASK_COMS_HANDLE_H

namespace arc_tasks{
  class Task_coms_handle{
  private:

    //====================================
    //
    // Guided Debris Request
    //
    //====================================

    ros::ServiceServer is_stuck_server;

    ros::Publisher guided_debris_pub;


    //====================================
    //
    //  Guided Debris Bidding
    //
    //====================================

    ros::Subscriber debris_auction_sub;

    ros::ServiceClient check_brain_debris;

    ros::Publisher debris_bid_pub;

    ros::Subscriber confirm_debris_bid_sub;


    //====================================
    //
    // Confirm Victim Request
    //
    //====================================

    ros::ServiceServer found_pot_vict_server;

    ros::Publisher found_pot_victim_pub;
    
    //====================================
    //
    // Confirm Victim Bidding
    //
    //====================================

    ros::Subscriber confirm_vict_auction_sub;

    ros::ServiceClient check_brain_conf_vict;

    ros::Publisher conf_vict_bid_pub;

    ros::Subscriber confirm_conf_vict_bid_sub;

    
  public:
    
  };

}


#endif //
