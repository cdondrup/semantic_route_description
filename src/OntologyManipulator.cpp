#include "route_description/OntologyManipulator.h"

#include "ontologenius/standard_service.h"

std::vector<std::string> OntologyManipulator::string2vector(const std::string& value)
{
  std::vector<std::string> result;
  bool eof = false;
  size_t begin = 0;
  do
  {
    size_t stop = value.find(" ", begin);
    std::string name = value.substr(begin , stop - begin);
    if(stop == std::string::npos)
      eof = true;
    else
      result.push_back(name);
    begin = stop + 1;
  }
  while(eof == false);

  return result;
}

std::string OntologyManipulator::getOn(const std::string& name, const std::string& property)
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::standard_service>("ontoloGenius/individual");

  ontologenius::standard_service srv;
  srv.request.action = "getOn";
  srv.request.param = name + ":" + property;

  cpt++;

  if(client.call(srv))
    return srv.response.value;
}

std::string OntologyManipulator::getFrom(const std::string& property, const std::string& name, const std::string& selector)
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::standard_service>("ontoloGenius/individual");

  ontologenius::standard_service srv;
  if(selector == "")
  {
    srv.request.action = "getFrom";
    srv.request.param = name + ":" + property;
  }
  else
  {
    srv.request.action = "select:getFrom";
    srv.request.param = selector + "=" + name + ":" + property;
  }

  cpt++;

  if(client.call(srv))
    return srv.response.value;
}

std::string OntologyManipulator::getUp(std::string& name)
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::standard_service>("ontoloGenius/individual");

  ontologenius::standard_service srv;
  srv.request.action = "getUp";
  srv.request.param = name;

  cpt++;

  if(client.call(srv))
    return srv.response.value;
}

std::string OntologyManipulator::getDown(std::string& name)
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::standard_service>("ontoloGenius/individual");

  ontologenius::standard_service srv;
  srv.request.action = "getDown";
  srv.request.param = name;

  cpt++;

  if(client.call(srv))
    return srv.response.value;
}

std::string OntologyManipulator::getRelatedFrom(const std::string& name)
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::standard_service>("ontoloGenius/individual");

  ontologenius::standard_service srv;
  srv.request.action = "getRelatedFrom";
  srv.request.param = name;

  cpt++;

  if(client.call(srv))
    return srv.response.value;
}

bool OntologyManipulator::close()
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::standard_service>("ontoloGenius/actions");
  ontologenius::standard_service srv;
  srv.request.action = "close";

  cpt++;

  if(!client.call(srv))
    return false;
  else
    return true;
}
