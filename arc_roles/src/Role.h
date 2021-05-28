

#ifndef ARC_ROLES_ROLE_H
#define ARC_ROLES_ROLE_H

#include <string>
#include "ros/ros.h"
#include "arc_msgs/QuerryRole.h"
#include <std_srvs/SetBool.h>
#include "std_msgs/Bool.h"

namespace arc_roles{
  class Role{
  private:
    float priority;
    int max_count;
    int curr_count;
    /*
    std::struct role_matrix{
	  float compute=0;
	  float memory=0;
	  float locomotion=0;
	  float size=0;
	  float victim_sensor_basic=0;
	  float victim_sensor_full=0;
	  float victim_tracker=0;
	  float robot_sensor=0;
	  float sonar_count=0;
	  float sonar_range=0;
	  float laser_count=0;
	  float frontier_finder=0;
	  float task_assign=0;
	  float planner=0;
	  float value=0;
	  float expendability=0;
	  float marker=0;
	  float marker_sensor=0;
    }matrix;

    */
    	ros::NodeHandle global_handle;
	ros::NodeHandle local_handle;

	ros::ServiceServer declare_role;

	ros::ServiceServer querry_role;

	void handle_declare_role();
	void handle_querry_role();

	void handle_declare(bool in_declare);
	float handle_querry(int in_bot_type);
  public:
	
	Role(std::string local);
	
	void run();

	bool declare_role_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
	bool querry_role_cb(arc_msgs::QuerryRole::Request &req, arc_msgs::QuerryRole::Response &res);

	void set_priority(float new_priority);

	void set_max_count(int new_max_count);

	void set_local_handle(std::string local_name);
    
  };
}


#endif //ARC_ROLES_ROLE_H
