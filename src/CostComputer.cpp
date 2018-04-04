#include "route_description/CostComputer.h"

#include <iostream>
#include <limits>
#include <map>

const struct Personna CostComputer::lambda_personna = {"lambda", {0.2,0.2,0.2,0.2,0.2}};
const struct Personna CostComputer::disabled_personna = {"disabled", {0.5,2.0,0.9,0.7,0.5}};
const struct Personna CostComputer::knowing_personna = {"knowing", {0.1,0.5,0.7,0.5,0.2}};
const struct Personna CostComputer::not_knowing_personna = {"notKnowing", {0.9,0.5,0.2,0.7,0.7}};
const struct Personna CostComputer::young_personna = {"young", {0.3,0.5,0.2,0.9,0.6}};
const struct Personna CostComputer::old_personna = {"old", {0.3,0.8,0.9,0.9,0.6}};

CostComputer::CostComputer(OntologyManipulator* onto, ros::NodeHandle* n)
{
  onto_ = onto;
  n_ = n;

  costs_.salient = 0;
  costs_.accessible = 0;
  costs_.confortable = 0;
  costs_.secured = 0;
  costs_.explicable = 0;
}

std::vector<std::string> CostComputer::splitPersonnas(std::string text)
{
  std::vector<std::string> personnas;
  size_t start = 0;
  size_t stop = 0;

  while(stop != std::string::npos)
  {
    stop = text.find("_", start);
    if(stop != std::string::npos)
    {
      std::string personna = text.substr(start, stop - start);
      personnas.push_back(personna);

      start = stop +1;
    }
  }
  std::string personna = text.substr(start, stop - start);
  personnas.push_back(personna);

  for(size_t i = 0; i < personnas.size(); i++)
    std::cout << personnas[i] << std::endl;

  return personnas;
}

void CostComputer::getParam(std::string personna_name)
{
  if(personna_name == lambda_personna.name)
    updateCosts(lambda_personna.costs);
  else if(personna_name == disabled_personna.name)
    updateCosts(disabled_personna.costs);
  else if(personna_name == knowing_personna.name)
    updateCosts(knowing_personna.costs);
  else if(personna_name == not_knowing_personna.name)
    updateCosts(not_knowing_personna.costs);
  else if(personna_name == young_personna.name)
    updateCosts(young_personna.costs);
  else if(personna_name == old_personna.name)
    updateCosts(old_personna.costs);
}

void CostComputer::updateCosts(struct Costs cost)
{
  if(cost.salient > costs_.salient)
    costs_.salient = cost.salient;

  if(cost.accessible > costs_.accessible)
    costs_.accessible = cost.accessible;

  if(cost.confortable > costs_.confortable)
    costs_.confortable = cost.confortable;

  if(cost.secured > costs_.secured)
    costs_.secured = cost.secured;

  if(cost.explicable > costs_.explicable)
    costs_.explicable = cost.explicable;
}

void CostComputer::putInRange()
{
  costs_.salient++;
  costs_.accessible++;
  costs_.confortable++;
  costs_.secured++;
  costs_.explicable++;
}

float CostComputer::getParamCost(std::string param)
{
  float cost = 1;
  if(param == "salient")
    cost = costs_.salient;
  else if(param == "insignifiant")
    cost = 1/costs_.salient;
  else if(param == "accessible")
    cost = costs_.accessible;
  else if(param == "inaccessible")
    cost = 1/costs_.accessible;
  else if(param == "confortable")
    cost = costs_.confortable;
  else if(param == "unconfortable")
    cost = 1/costs_.confortable;
  else if(param == "secured")
    cost = costs_.confortable;
  else if(param == "unsecured")
    cost = 1/costs_.confortable;
  else if(param == "explicable")
    cost = costs_.confortable;
  else if(param == "unexplicable")
    cost = 1/costs_.confortable;

  if((cost > 2) || (cost < 1/2.))
    return std::numeric_limits<float>::infinity();
  else
    return 1/cost;
}

std::vector<float> CostComputer::compute(routes_t& routes, std::string personnas)
{
  std::vector<std::string> personnas_splitted = splitPersonnas(personnas);
  for(size_t i = 0; i < personnas_splitted.size(); i++)
    getParam(personnas_splitted[i]);

  putInRange();

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
