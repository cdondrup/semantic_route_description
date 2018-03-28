#ifndef CORRIDORPATHFINDER_H
#define CORRIDORPATHFINDER_H

#include "route_description/OntologyManipulator.h"

#include <vector>
#include <string>

class CorridorPathfinder
{
public:
  CorridorPathfinder(OntologyManipulator* onto) {onto_ = onto; }
  ~CorridorPathfinder(){}

  std::vector<std::vector<std::string>> find(std::string& my_corridor, std::string& goal_corridor, size_t& max_step);

  void displayRoutes(std::vector<std::vector<std::string>> routes);
  void displayRoute(std::vector<std::string> route);

private:
  OntologyManipulator* onto_;

  void selectIntersections(std::vector<std::string>& markers);
};

#endif
