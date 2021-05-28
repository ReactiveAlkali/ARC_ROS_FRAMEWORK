#include "ros/ros.h"
#include "Role.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "lieutenent_node");
  arc_roles::Role lieutenent("LIEUTENENT");

  lieutenent.set_priority(1.0);
  lieutenent.set_max_count(1);

  lieutenent.run();
  return 0;
}
