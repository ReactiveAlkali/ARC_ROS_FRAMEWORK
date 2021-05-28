#include "Task_center.h"
#include "ros/ros.h"

int main(int argc, char ** argv) {
    ros::init(argc,argv, "task_center");
    arc_tasks::Task_center task_center;
    ros::spin();

    return 0;
}
