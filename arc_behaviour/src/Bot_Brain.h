

#ifndef ARC_BEHAVIOUR_BOT_BRAIN_H
#define ARC_BEHAVIOUR_BOT_BRAIN_H

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "arc_msgs/QuerryTask.h"
#include "arc_msgs/TaskService.h"
#include "arc_msgs/ToggleList.h"


namespace arc_behaviour{

  class Bot_Brain{
    private:

    int role_type;
    int bot_type;
    int usefulness;

    
    bool enabled;
    
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle;

    //===================================
    //
    //  Task Server
    //
    //===================================

    ros::ServiceServer in_toggle_list_server;

    //===================================
    //
    // toggle behaviours
    //
    //===================================

    //client to send for toggling random wandering
    ros::ServiceClient toggle_random_wander_client;

    //client to toggle cleaning debris;
    ros::ServiceClient toggle_clean_debris_client;

    //client to toggle handle marker
    ros::ServiceClient toggle_handle_marker_client;

    //client to toggle moving to goal
    ros::ServiceClient toggle_move_to_goal_client;

    ros::ServiceServer toggle_server;

    //=================================
    //
    //QUERRY FOR EFFICENCY
    //
    //=================================

    //client to querry for a leader role
    ros::ServiceClient querry_leader_client;

    //client to querry for a verifier role
    ros::ServiceClient querry_verifier_client;

    //client to querry for an explorer role
    ros::ServiceClient querry_explorer_client;


    //====================================
    //
    //DECLARE YOUR ROLE
    //
    //====================================
   
    //client to declare the bot a leader role
    ros::ServiceClient declare_leader_client;

    //client to declare the bot a verifier
    ros::ServiceClient declare_verifier_client;

    //client to declare the bot an explorer
    ros::ServiceClient declare_explorer_client;


    bool leader_role;
    
    void toggle(bool state);

    void setRole();

    void setLeaderRole();

    void setVerifierRole();

    void setExplorerRole();

    void newJob();

    int get_curr_task();
    
    public:

    Bot_Brain();

    //bool newJob(Somthing goes here);

      bool toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

      bool in_toggle_list_cb(arc_msgs::ToggleList::Request &req, arc_msgs::ToggleList::Response &res);

     
  
    void run();
  };
}

#endif //
