
#include "ros/ros.h"
#include "Role.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "general_node");
  arc_roles::Role general("GENERAL");

  general.set_priority(5.0);
  general.set_max_count(1);
  general.set_min_count(1);

  general.run();
  return 0;
}
