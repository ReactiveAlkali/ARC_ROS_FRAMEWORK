/**
* CLASS: RandomWanderMS
* DATE: 20/02/17
* AUTHOR: Kyle Morris
* DESCRIPTION: Random wander behaviour that sends various random navigation goals to navigation stack.
*/
#ifndef ARC_BEHAVIOUR_TOGGLETEST_H
#define ARC_BEHAVIOUR_TOGGLETEST_H

#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include "std_srvs/SetBool.h"

namespace arc_behaviour {
    class Toggle_Test {
    private:
        ros::ServiceServer toggle_server;

        ros::NodeHandle global_handle;
        ros::NodeHandle local_handle;

	bool enabled;

        void toggle(bool state);
    public:
        Toggle_Test();

        void run();

        bool toggle_cb(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

    };
}

#endif //ARC_BEHAVIOUR_TOGGLETEST_H
