#include "ros/ros.h"
#include "Role.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "explorer_node");
  arc_roles::Role explorer("EXPLORER");

  explorer.set_priority(0.1);
  explorer.set_max_count(10);
  explorer.set_min_count(3);

  explorer.run();
  return 0;
}
