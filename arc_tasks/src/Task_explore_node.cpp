#include "Task_explore.h"

int main(int argc, char ** argv) {
    ros::init(argc,argv, "task_explore");
    arc_tasks::Task_explore task_explore;
    ros::spin();

    return 0;
}
