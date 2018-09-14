#include "route_description/CorridorPathfinder.h"

#include <iostream>

routes_t CorridorPathfinder::find(std::string& my_corridor, std::string& goal_corridor, size_t& max_step)
{
  routes_t routes;
  bool found = false;
  size_t progress = 0;

  {
    std::vector<std::string> tmp;
    tmp.push_back(my_corridor);
    routes.push_back(tmp);
  }

  size_t route_i, marker_i, i, j;

  if(my_corridor == goal_corridor)
    found = true;

  size_t cpt = 0;
  while(!found)
  {
    routes_t tmp_routes;
    size_t routes_size = routes.size();
    for(route_i = 0; route_i < routes_size; route_i++)
    {
      size_t size = routes[route_i].size() - 1;
      std::vector<std::string> markers = onto_->individuals.getFrom("isAlong", routes[route_i][size], "pathIntersection");

      for(marker_i = 0; marker_i < markers.size(); marker_i++)
      {
        std::vector<std::string> corridors = onto_->individuals.getOn(markers[marker_i], "isAlong");

        for(i = 0; i < corridors.size();)
        {
          bool erase = false;
          for(j = 0; j < routes_size; j++)
            if(std::find(routes[j].begin(), routes[j].end(), corridors[i]) != routes[j].end())
            {
              erase = true;
              break;
            }

          if(erase == true)
            corridors.erase(corridors.begin() + i);
          else
            i++;
        }

        for(size_t i = 0; i < corridors.size(); i++)
        {
          progress++;
          route_t tmp = routes[route_i];
          tmp.push_back(markers[marker_i]);
          tmp.push_back(corridors[i]);
          tmp_routes.push_back(tmp);
          if(corridors[i] == goal_corridor)
            found = true;
        }
      }
    }
    routes.swap(tmp_routes);
    routes_size = routes.size();

    if(progress == 0)
      found = true;
    else
      progress = 0;

    cpt ++;
    if(cpt >= max_step)
      found = true;
  }

  if(cpt < max_step)
    max_step = cpt;

  for(size_t i = 0; i < routes.size();)
  {
    if(routes[i][routes[i].size() - 1] != goal_corridor)
      routes.erase(routes.begin() + i);
    else
      i++;
  }

  return routes;
}

void CorridorPathfinder::displayRoutes(routes_t& routes)
{
  for(size_t route_i = 0; route_i < routes.size(); route_i++)
    displayRoute(routes[route_i]);
}

void CorridorPathfinder::displayRoute(route_t& route)
{
  for(size_t i = 0; i < route.size(); i++)
    std::cout << " -- " << route[i];
  std::cout << std::endl;
}
