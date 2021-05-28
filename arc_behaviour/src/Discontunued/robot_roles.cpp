#include "robot_roles.h"

#define MAX_QUEUE_SIZE 1000
#define DEFAULT_RATE 10

using namespace arc_behaviour;

Robot_Roles::Robot_Roles(std::String name, matrix *in_matrix){
  ros::NodeHandle global_handle;
  ros::NodeHandle local_handle("robot_roles");

  this->global_handle = global_handle;
  this->local_handle = local_handle;

  
  role_matrix->compute = in_matrix->compute;
  role_matrix->memory = in_matrix->memory;
  role_matrix->locomotion = in_matrix->locomotion;
  role_matrix->size = in_matrix->size;
  role_matrix->victim_sensor_basic = in_matrix->victim_sensor_basic;
  role_matrix->victim_sensor_full = in_matrix->victim_sonsor_full;
  role_matrix->robot_sensor = in_matrix->robot_sensor;
  role_matrix->sonar_count = in_matrix->sonar_count;
  role_matrix->sonar_range = in_matrix->sonar_range;
  role_matrix->laser_count = in_matrix->laser_count;
  role_matrix->frontier_finder = in_matrix->frontier_finder;
  role_matrix->task_assign = in_matrix->task_assign;
  role_matrix->planner = in_matirx->planner;
  role_matrix->value = in_matrix->value;
  role_matrix->marker = in_matrix->marker;
  role_matrix->marker_sensor = in_matrix->marker_sensor
}

void Robot_Roles::set_max_count(int count){
  this->max_count = count;
}

void Robot_Roles::run(){
  ros::Rate r(DEFAULT_RATE);
  while(ros::ok()){
    ros::spinOnce():
    r.sleep():
  }
}
