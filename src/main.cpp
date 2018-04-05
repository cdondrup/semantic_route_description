#include <iostream>
#include "ros/ros.h"

#include <vector>
#include <string>

#include "route_description/PathFinder.h"
#include "semantic_route_description/SemanticRoute.h"
#include "semantic_route_description/Route.h"

ros::NodeHandle* n_;

bool route_handle(semantic_route_description::SemanticRoute::Request  &req,
                  semantic_route_description::SemanticRoute::Response &res)
{
  PathFinder finder(n_);
  finder.find(req.from, req.to, req.personna);

  routes_t tmp = finder.getRoutes();
  for(size_t i = 0; i < tmp.size(); i++)
  {
    semantic_route_description::Route tmp_route;
    tmp_route.route = tmp[i];
    res.routes.push_back(tmp_route);
  }
  res.costs = finder.getCosts();

  return true;
}

bool routeRegion_handle(semantic_route_description::SemanticRoute::Request  &req,
                        semantic_route_description::SemanticRoute::Response &res)
{
  PathFinder finder(n_);
  finder.findRegions(req.from, req.to, req.personna);

  routes_t tmp = finder.getRoutes();
  for(size_t i = 0; i < tmp.size(); i++)
  {
    semantic_route_description::Route tmp_route;
    tmp_route.route = tmp[i];
    res.routes.push_back(tmp_route);
  }
  res.costs = finder.getCosts();

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "route_description");

  ros::NodeHandle n;
  n_ = &n;

  ros::service::waitForService("ontoloGenius/arguer", -1);
  ros::ServiceServer service = n.advertiseService("semantic_route_description/getRoute", route_handle);
  ros::ServiceServer route_service = n.advertiseService("semantic_route_description/getRouteRegion", routeRegion_handle);
  ROS_DEBUG("semantic_route_description ready");

  ros::spin();

  ROS_DEBUG("KILL semantic_route_description");

  return 0;
}
