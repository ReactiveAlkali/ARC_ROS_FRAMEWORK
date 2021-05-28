#include "ros/ros.h"
#include "Toggle_Test.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "toggle_test");
    arc_behaviour::Toggle_Test toggle_test;
    toggle_test.run();
    return 0;
}
