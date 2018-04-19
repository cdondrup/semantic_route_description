 [![License][License-Image]][License-Url]
 [![Dependency Status][Dependency-Image]][Dependency-Url]


## Launch the semantic route description

Fisrt, launch the ontoloGenius with the semantic_route_description's files:
```
$ roslaunch semantic_route_description route_ontology.launch
```
A graphical user interface will be launched, close ontoloGenius with the button "Close Ontology" at the bottom left.

Now, you can run the semantic_route_description:
```
$ rosrun semantic_route_description description
```

## Call the route description

The services definition is the following:
```
string from
string to
string persona
bool signpost
---
Route[] routes
float32[] costs
string[] goals
```

The service will give you three arrays in return. All arrays are the same size and give you a route (string array), a cost for each corresponding route and the final location for each corresponding route.

### Basic usage

To get the route from **ff_c3_begin** to **reima**, call the getRoute service as follows:
```
$ rosservice call /semantic_route_description/get_route "{from_: 'ff_c3_begin', to: 'reima', persona: 'lambda'}"
```

To get the route from **ff_c3_begin** to **reima** only with regions and interfaces, call the getRouteRegion service as follows:
```
$ rosservice call /semantic_route_description/get_route_region "{from_: 'ff_c3_begin', to: 'reima', persona: 'lambda'}"
```

### Persona usage

For the persona parameter, you have six types of persona that you can combine with the `_` delimiter:
 - lambda
 - disabled
 - knowing
 - notKnowing
 - young
 - old

### Signpost usage

The signpost parameter can be set to true to allow the route description to find routes that do not go directly to the requested location but that go to a sign bound to that location.
If you use this option, you can use the `goals` result which indicates the final destination of each route.


[License-Url]: https://opensource.org/licenses/Apache-2.0
[License-Image]: https://img.shields.io/badge/License-Apache%202.0-blue.svg
[Dependency-Image]: https://img.shields.io/badge/dependencies-ontoloGenius-1eb0fc.svg
[Dependency-Url]: https://github.com/sarthou/ontologenius
