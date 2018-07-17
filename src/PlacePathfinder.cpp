#include "route_description/PlacePathfinder.h"
#include "route_description/CorridorPathfinder.h"

routes_t PlacePathfinder::find(std::string& my_place, std::string& goal_place, std::string& region)
{
  routes_t routes;
  CorridorPathfinder corridor_pathfinder(onto_);

  std::vector<std::string> from_corridors = onto_->individuals.getOn(my_place, "isAlong");
  removeNotInRegion(from_corridors, region);
  std::vector<std::string> to_corridors = onto_->individuals.getOn(goal_place, "isAlong");
  removeNotInRegion(to_corridors, region);

  size_t min = -1;
  for(size_t from = 0; from < from_corridors.size(); from++)
    for(size_t to = 0; to < to_corridors.size(); to++)
    {
      std::vector<std::vector<std::string>> corridors = corridor_pathfinder.find(from_corridors[from], to_corridors[to], min);
      for(size_t i = 0; i < corridors.size(); i++)
        routes.push_back(corridors[i]);
    }

  min = 2*min + 1;
  for(size_t i = 0; i < routes.size();)
    if(routes[i].size() > min+1)
      routes.erase(routes.begin() + i);
    else
      i++;

  return routes;
}

void PlacePathfinder::removeNotInRegion(std::vector<std::string>& corridors, std::string& region)
{
  for(size_t i = 0; i < corridors.size(); )
  {
    std::vector<std::string> regions = onto_->individuals.getOn(corridors[i], "isIn");
    if(std::find(regions.begin(), regions.end(), region) == regions.end())
      corridors.erase(corridors.begin() + i);
    else
      i++;
  }
}
