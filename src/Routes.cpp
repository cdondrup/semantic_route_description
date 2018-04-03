#include "route_description/Routes.h"

void append(routes_t& base, routes_t& add)
{
  routes_t tmp;
  for(size_t i = 0; i < add.size(); i++)
    tmp.insert(tmp.end(), base.begin(), base.end());

  for(size_t base_i = 0; base_i < base.size(); base_i++)
    for(size_t add_i = 0; add_i < add.size(); add_i++)
      tmp[base_i + add_i].insert(tmp[base_i + add_i].end(), add[add_i].begin(), add[add_i].end());

  base = tmp;
}

void append(routes_t& base, std::string& add)
{
  for(size_t i = 0; i < base.size(); i++)
    base[i].push_back(add);
}
