#ifndef ROUTES_H
#define ROUTES_H

#include <vector>
#include <string>

typedef std::vector<std::string> route_t;
typedef std::vector<std::vector<std::string>> routes_t;

void append(routes_t& base, routes_t& add);
void append(routes_t& base, std::string& add);

#endif
