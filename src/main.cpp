#include "route_description/OntologyManipulator.h"
#include "route_description/RegionPathfinder.h"
#include "route_description/PlacePathfinder.h"
#include "route_description/PlaceToRegion.h"
#include "route_description/CostComputer.h"

#include <iostream>
#include "ros/ros.h"

#include <vector>
#include <string>

#include <chrono>
#include <ctime>

std::string goal = "reima";
std::string place_pose = "ff_c3_begin";//"atm_2";//"ff_c4_begin";
std::string place_goal = "HandM";//"ff_c1_begin";

//roslaunch pepper_bringup pepper_full.launch nao_ip:=mummer.laas.fr roscore_ip:=127.0.0.1 network_interface:=enp0s31f6

typedef std::vector<std::vector<std::string>> routes_t;

void append(routes_t& base, routes_t& add)
{
  routes_t tmp;
  for(size_t i = 0; i < add.size(); i++)
    tmp.insert(tmp.end(), base.begin(), base.end());

  for(size_t base_i = 0; base_i < base.size(); base_i++)
    for(size_t add_i = 0; add_i < add.size(); add_i++)
      tmp[base_i + add_i].insert(tmp[base_i + add_i].end(), add[add_i].begin(), add[add_i].end());

  base = tmp;
}

void append(routes_t& base, std::string& add)
{
  for(size_t i = 0; i < base.size(); i++)
    base[i].push_back(add);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "route_description");

  ros::NodeHandle n;
  OntologyManipulator onto(&n);
  CostComputer cost(&onto, &n);
  PlaceToRegion place_to_region(&onto);

  float salient;
  if (n.getParam(ros::this_node::getName() + "/salient", salient))
    std::cout << "salient " << salient << std::endl;

  ros::service::waitForService("ontoloGenius/arguer", -1);
  onto.close();

  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  //////////////////////////////
  //  get current position and goal
  /////////////////////////////

  std::vector<std::string> from_region = place_to_region.place2region(place_pose);
  std::vector<std::string> to_region = place_to_region.place2region(place_goal);

  std::cout << "from ";
  for(size_t i = 0; i < from_region.size(); i++)
    std::cout << from_region[i] << " ";
  std::cout << std::endl;

  std::cout << "to ";
  for(size_t i = 0; i < to_region.size(); i++)
    std::cout << to_region[i] << " ";
  std::cout << std::endl;

  //////////////////////////////
  //  get region routes
  /////////////////////////////

  RegionPathfinder region_pathfinder(&onto);
  std::vector<routes_t> routes;
  for(size_t from = 0; from < from_region.size(); from++)
    for(size_t to = 0; to < to_region.size(); to++)
      routes.push_back(region_pathfinder.find(from_region[from], to_region[to]));

  //////////////////////////////
  //  add start position and goal
  /////////////////////////////
  for(size_t i = 0; i < routes.size(); i++)
    for(size_t route_i = 0; route_i < routes[i].size(); route_i++)
    {
      routes[i][route_i].insert(routes[i][route_i].begin(), place_pose);
      routes[i][route_i].push_back(place_goal);
    }

  std::cout << routes.size() << " routes are : " << std::endl;
  for(size_t i = 0; i < routes.size(); i++)
    region_pathfinder.displayRoutes(routes[i]);

  std::cout << "NB : " << onto.nb() << std::endl;
  //onto.reset();

  //////////////////////////////
  //  get paths in routes
  /////////////////////////////
  std::map<std::string, routes_t> place2place;
  PlacePathfinder place_pathfinder(&onto);

  for(size_t i = 0; i < routes.size(); i++)
  {
    for(size_t route_i = 0; route_i < routes[i].size(); route_i++)
    {
      for(size_t place_i = 0; place_i + 2 < routes[i][route_i].size(); place_i+=2)
      {
        std::cout << routes[i][route_i][place_i] << " " << routes[i][route_i][place_i + 2] << std::endl;
        std::string id = routes[i][route_i][place_i] + ":" + routes[i][route_i][place_i + 1] + ":" + routes[i][route_i][place_i + 2];
        if(place2place.find(id) == place2place.end())
        {
          routes_t corridors = place_pathfinder.find(routes[i][route_i][place_i], routes[i][route_i][place_i + 2], routes[i][route_i][place_i + 1]);
          place2place[id] = corridors;
        }
        std::cout << "NB : " << onto.nb() << std::endl;
        //onto.reset();
      }
      std::cout<< std::endl;
    }
    std::cout<< std::endl;
  }

  //////////////////////////////
  //  get completed paths
  /////////////////////////////
  routes_t completed_routes;
  for(size_t i = 0; i < routes.size(); i++)
  {
    for(size_t route_i = 0; route_i < routes[i].size(); route_i++)
    {
      routes_t tmp_routes;
      {
        std::vector<std::string> tmp;
        tmp.push_back(routes[i][route_i][0]);
        tmp_routes.push_back(tmp);
      }
      for(size_t place_i = 0; place_i + 2 < routes[i][route_i].size(); place_i+=2)
      {
        std::string id = routes[i][route_i][place_i] + ":" + routes[i][route_i][place_i + 1] + ":" + routes[i][route_i][place_i + 2];

        if(place2place[id].size() != 0)
          append(tmp_routes, place2place[id]);
        else
          append(tmp_routes, routes[i][route_i][place_i+1]);
        append(tmp_routes, routes[i][route_i][place_i+2]);
      }
      completed_routes.insert(completed_routes.end(), tmp_routes.begin(), tmp_routes.end());
    }
  }

  std::cout << " completed routes are : " << std::endl;
  region_pathfinder.displayRoutes(completed_routes);

  cost.compute(completed_routes);

  end = std::chrono::system_clock::now();
  int elapsed_seconds = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  std::cout << "finished computation at " << std::ctime(&end_time)
            << "elapsed time: " << elapsed_seconds << "s\n";

  ROS_DEBUG("KILL route_description");

  return 0;
}
