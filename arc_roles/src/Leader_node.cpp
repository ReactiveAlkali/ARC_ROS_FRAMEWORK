
#include "ros/ros.h"
#include "Role.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "leader_node");
  arc_roles::Role leader("LEADER");

  leader.set_priority(5.0);
  leader.set_max_count(1);
  leader.set_min_count(1);

  leader.run();
  return 0;
}
