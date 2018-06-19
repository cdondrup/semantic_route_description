#include "route_description/PlaceToRegion.h"

#include <iostream>

std::vector<std::string> PlaceToRegion::place2region(std::string place)
{
  std::vector<std::string> paths = onto_->getOn(place, "isAlong");

  std::vector<std::string> regions;
  for(size_t i = 0; i < paths.size(); i++)
  {
    std::vector<std::string> tmp = onto_->getOn(paths[i], "isIn");
    if(tmp.size())
      if(std::find(regions.begin(), regions.end(), tmp[0]) == regions.end())
        regions.push_back(tmp[0]);
  }

  return regions;
}
