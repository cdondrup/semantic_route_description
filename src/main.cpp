#include "route_description/PathFinder.h"

#include <iostream>
#include "ros/ros.h"

#include <vector>
#include <string>

std::string goal = "reima";
std::string place_pose = "ff_c3_begin";//"atm_2";//"ff_c4_begin";
std::string place_goal = "HandM";//"ff_c1_begin";

//roslaunch pepper_bringup pepper_full.launch nao_ip:=mummer.laas.fr roscore_ip:=127.0.0.1 network_interface:=enp0s31f6


int main(int argc, char** argv)
{
  ros::init(argc, argv, "route_description");

  ros::NodeHandle n;

  ros::service::waitForService("ontoloGenius/arguer", -1);

  PathFinder finder(&n);
  finder.find(place_pose, place_goal);

  ROS_DEBUG("KILL route_description");

  return 0;
}
