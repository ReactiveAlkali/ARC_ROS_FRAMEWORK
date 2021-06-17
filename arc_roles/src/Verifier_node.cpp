#include "ros/ros.h"
#include "Role.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "verifier_node");
  arc_roles::Role verifier("VERIFIER");

  verifier.set_priority(1.0);
  verifier.set_max_count(3);
  verifier.set_min_count(1);

  verifier.run();
  return 0;
}
