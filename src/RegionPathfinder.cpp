#include "route_description/RegionPathfinder.h"

#include <iostream>

std::vector<std::vector<std::string>> RegionPathfinder::find(std::string my_region, std::string goal_region)
{
  std::vector<std::vector<std::string>> routes;
  bool found = false;

  {
    std::vector<std::string> tmp;
    tmp.push_back(my_region);
    routes.push_back(tmp);
  }

  if(my_region == goal_region)
    found = true;

  while(!found)
  {
    std::vector<std::vector<std::string>> tmp_routes;
    for(size_t route_i = 0; route_i < routes.size(); route_i++)
    {
      size_t size = routes[route_i].size() - 1;
      std::vector<std::string> interfaces = onto_->string2vector(onto_->getFrom("isIn", routes[route_i][size]));

      for(size_t inter = 0; inter < interfaces.size(); inter++)
      {
        std::vector<std::string> regions = onto_->string2vector(onto_->getOn(interfaces[inter], "isIn"));

        for(size_t i = 0; i < regions.size();)
          if(std::find(routes[route_i].begin(), routes[route_i].end(), regions[i]) != routes[route_i].end())
            regions.erase(regions.begin() + i);
          else
            i++;

        for(size_t i = 0; i < regions.size(); i++)
        {
          std::vector<std::string> tmp = routes[route_i];
          tmp.push_back(interfaces[inter]);
          tmp.push_back(regions[i]);
          tmp_routes.push_back(tmp);
          if(regions[i] == goal_region)
            found = true;
        }
      }
    }
    routes = tmp_routes;
  }

  for(size_t i = 0; i < routes.size();)
  {
    if(routes[i][routes[i].size() - 1] != goal_region)
      routes.erase(routes.begin() + i);
    else
      i++;
  }

  return routes;
}

void RegionPathfinder::displayRoutes(std::vector<std::vector<std::string>> routes)
{
  for(size_t route_i = 0; route_i < routes.size(); route_i++)
    displayRoute(routes[route_i]);
}

void RegionPathfinder::displayRoute(std::vector<std::string> route)
{
  for(size_t i = 0; i < route.size(); i++)
    std::cout << " -- " << route[i];
  std::cout << std::endl;
}
