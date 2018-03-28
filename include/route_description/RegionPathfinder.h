#ifndef REGIONPATHFINDER_H
#define REGIONPATHFINDER_H

#include "route_description/OntologyManipulator.h"

#include <vector>
#include <string>

class RegionPathfinder
{
public:
  RegionPathfinder(OntologyManipulator* onto) {onto_ = onto; }
  ~RegionPathfinder(){}

  std::vector<std::vector<std::string>> find(std::string my_region, std::string goal_region);

  void displayRoutes(std::vector<std::vector<std::string>> routes);
  void displayRoute(std::vector<std::string> route);

private:
  OntologyManipulator* onto_;
};

#endif
