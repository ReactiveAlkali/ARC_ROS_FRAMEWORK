#include "Task_unguided_debris.h"

int main(int argc, char ** argv) {
    ros::init(argc,argv, "task_unguided_debris");
    arc_tasks::Task_unguided_debris task_unguided_debris;
    ros::spin();

    return 0;
}
