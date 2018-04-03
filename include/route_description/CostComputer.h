#ifndef COSTCOMPUTER_H
#define COSTCOMPUTER_H

#include "route_description/OntologyManipulator.h"
#include "route_description/Routes.h"

#include "ros/ros.h"
#include <string>
#include <vector>

class CostComputer
{
public:
  CostComputer(OntologyManipulator* onto, ros::NodeHandle* n);
  ~CostComputer(){}

  std::vector<float> compute(routes_t& routes);

private:
  OntologyManipulator* onto_;
  ros::NodeHandle* n_;

  float salient_;
  float accessible_;
  float confortable_;
  float secured_;
  float explicable_;

  void getParam(std::string name, float& param);
  void getParams();
  float getParamCost(std::string param);
};

#endif
