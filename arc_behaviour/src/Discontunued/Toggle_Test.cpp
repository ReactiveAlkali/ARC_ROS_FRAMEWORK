#include <std_srvs/Trigger.h>
#include "Toggle_Test.h"
#define MAX_QUEUE_SIZE 1000
#define DEFAULT_RATE 10

using namespace arc_behaviour;

Toggle_Test::Toggle_Test() {
    ros::NodeHandle global_handle;
    ros::NodeHandle local_handle("toggle_test");

    this->global_handle = global_handle;
    this->local_handle = local_handle;
    ROS_INFO("Setting up toggle test");


    this->toggle_server = this->local_handle.advertiseService("toggle", &Toggle_Test::toggle_cb, this);

    this->toggle(false);
}

void Toggle_Test::run() {
    ros::Rate r(DEFAULT_RATE);
    //set a timer to call random generation of goals every once in a while

    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

}

bool Toggle_Test::toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    this->toggle(req.data);
    return true;
}

void Toggle_Test::toggle(bool state) {
    this->enabled = state;
    if(this->enabled) {
        ROS_INFO("Toggle_Test has been enabled.");
        //send an immediate goal right when we start.
    } else {
        ROS_INFO("Toggle_Test has been disabled.");
    }
}

