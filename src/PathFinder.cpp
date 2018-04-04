#include "route_description/PathFinder.h"

#include <iostream>
#include <chrono>
#include <ctime>

#include "route_description/PlaceToRegion.h"
#include "route_description/RegionPathfinder.h"
#include "route_description/PlacePathfinder.h"
#include "route_description/CostComputer.h"

PathFinder::PathFinder(ros::NodeHandle* n) : onto_(n)
{
  n_ = n;
  onto_.close();
}

void PathFinder::find(std::string from_place, std::string to_place)
{
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  to_regions(from_place, to_place);
  getRegionRoutes();
  appendFromAndTo(from_place, to_place);

  createPlace2Place();
  getCompleteRoutes();
  printFinalRoutes();

  computeCost();

  end = std::chrono::system_clock::now();
  int elapsed_seconds = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  std::cout << "finished computation at " << std::ctime(&end_time)
            << "elapsed time: " << elapsed_seconds << "s\n";
}

void PathFinder::to_regions(std::string from_place, std::string to_place)
{
  PlaceToRegion place_to_region(&onto_);

  from_region_ = place_to_region.place2region(from_place);
  to_region_ = place_to_region.place2region(to_place);

  std::cout << "From :";
  for(size_t i = 0; i < from_region_.size(); i++)
    std::cout << from_region_[i] << " ";
  std::cout << std::endl;

  std::cout << "To   :";
  for(size_t i = 0; i < to_region_.size(); i++)
    std::cout << to_region_[i] << " ";
  std::cout << std::endl;
}

void PathFinder::getRegionRoutes()
{
  RegionPathfinder region_pathfinder(&onto_);
  for(size_t from = 0; from < from_region_.size(); from++)
    for(size_t to = 0; to < to_region_.size(); to++)
      routes_.push_back(region_pathfinder.find(from_region_[from], to_region_[to]));
}

void PathFinder::appendFromAndTo(std::string from_place, std::string to_place)
{
  for(size_t i = 0; i < routes_.size(); i++)
    for(size_t route_i = 0; route_i < routes_[i].size(); route_i++)
    {
      routes_[i][route_i].insert(routes_[i][route_i].begin(), from_place);
      routes_[i][route_i].push_back(to_place);
    }
}

void PathFinder::createPlace2Place()
{
  PlacePathfinder place_pathfinder(&onto_);

  for(size_t i = 0; i < routes_.size(); i++)
  {
    for(size_t route_i = 0; route_i < routes_[i].size(); route_i++)
    {
      for(size_t place_i = 0; place_i + 2 < routes_[i][route_i].size(); place_i+=2)
      {
        std::string id = routes_[i][route_i][place_i] + ":" + routes_[i][route_i][place_i + 1] + ":" + routes_[i][route_i][place_i + 2];
        if(place2place_.find(id) == place2place_.end())
        {
          routes_t corridors = place_pathfinder.find(routes_[i][route_i][place_i], routes_[i][route_i][place_i + 2], routes_[i][route_i][place_i + 1]);
          place2place_[id] = corridors;
        }
      }
    }
  }
}

void PathFinder::getCompleteRoutes()
{
  for(size_t i = 0; i < routes_.size(); i++)
  {
    for(size_t route_i = 0; route_i < routes_[i].size(); route_i++)
    {
      routes_t tmp_routes;
      {
        std::vector<std::string> tmp;
        tmp.push_back(routes_[i][route_i][0]);
        tmp_routes.push_back(tmp);
      }
      for(size_t place_i = 0; place_i + 2 < routes_[i][route_i].size(); place_i+=2)
      {
        std::string id = routes_[i][route_i][place_i] + ":" + routes_[i][route_i][place_i + 1] + ":" + routes_[i][route_i][place_i + 2];

        if(place2place_[id].size() != 0)
          append(tmp_routes, place2place_[id]);
        else
          append(tmp_routes, routes_[i][route_i][place_i+1]);
        append(tmp_routes, routes_[i][route_i][place_i+2]);
      }
      completed_routes_.insert(completed_routes_.end(), tmp_routes.begin(), tmp_routes.end());
    }
  }
}

void PathFinder::printFinalRoutes()
{
  std::cout << " completed routes are : " << std::endl;
  RegionPathfinder region_pathfinder(&onto_);
  region_pathfinder.displayRoutes(completed_routes_);
}

void PathFinder::computeCost()
{
  CostComputer cost(&onto_, n_);
  costs_ = cost.compute(completed_routes_);

  for(size_t route_i = 0; route_i < costs_.size(); route_i++)
    std::cout << costs_[route_i] << std::endl;
}
