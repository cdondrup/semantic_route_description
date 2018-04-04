#include "route_description/CostComputer.h"

#include <iostream>
#include <limits>
#include <map>

CostComputer::CostComputer(OntologyManipulator* onto, ros::NodeHandle* n)
{
  onto_ = onto;
  n_ = n;

  salient_ = 0;
  accessible_ = 0;
  confortable_ = 0;
  secured_ = 0;
  explicable_ = 0;

  getParams();
}

void CostComputer::getParam(std::string name, float& param)
{
  if (n_->getParam(ros::this_node::getName() + "/" + name, param))
    std::cout << name << " : " << param << std::endl;
  else
    std::cout << name << " will take default value : " << param << std::endl;
}

void CostComputer::getParams()
{
  getParam("salient", salient_); salient_++;
  getParam("accessible", accessible_); accessible_++;
  getParam("confortable", confortable_); confortable_++;
  getParam("secured", secured_); secured_++;
  getParam("explicable", explicable_); explicable_++;
}

float CostComputer::getParamCost(std::string param)
{
  float cost = 1;
  if(param == "salient")
    cost = salient_;
  else if(param == "insignifiant")
    cost = 1/salient_;
  else if(param == "accessible")
    cost = accessible_;
  else if(param == "inaccessible")
    cost = 1/accessible_;
  else if(param == "confortable")
    cost = confortable_;
  else if(param == "unconfortable")
    cost = 1/confortable_;
  else if(param == "secured")
    cost = confortable_;
  else if(param == "unsecured")
    cost = 1/confortable_;
  else if(param == "explicable")
    cost = confortable_;
  else if(param == "unexplicable")
    cost = 1/confortable_;

  if((cost > 2) || (cost < 1/2.))
    return std::numeric_limits<float>::infinity();
  else
    return 1/cost;
}

std::vector<float> CostComputer::compute(routes_t& routes)
{
  std::vector<float> result;
  for(size_t route_i = 0; route_i < routes.size(); route_i++)
    result.push_back(routes[route_i].size());

  std::map<std::string, float> elementCost;
  std::vector<std::string> havingCost = onto_->string2vector(onto_->getRelatedFrom("hasCost"));
  for(size_t i = 0; i < havingCost.size(); i++)
  {
    elementCost[havingCost[i]] = 1.0;
    std::vector<std::string> cost = onto_->string2vector(onto_->getOn(havingCost[i], "hasCost"));
    for(size_t j = 0; j < cost.size(); j++)
    {
      elementCost[havingCost[i]] = elementCost[havingCost[i]] * getParamCost(cost[j]);
    }

    for(size_t route_i = 0; route_i < routes.size(); route_i++)
    {
      if(std::find(routes[route_i].begin(), routes[route_i].end(), havingCost[i]) != routes[route_i].end())
        result[route_i] = result[route_i]*elementCost[havingCost[i]];
    }
  }

  return result;
}
