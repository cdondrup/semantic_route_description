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

To get the route from **ff_c3_begin** to **reima**, call the getRoute service as follows:
```
$ rosservice call /semantic_route_description/get_route "{from_: 'ff_c3_begin', to: 'reima', personna: 'lambda'}"
```

To get the route from **ff_c3_begin** to **reima** only with regions and interfaces, call the getRouteRegion service as follows:
```
$ rosservice call /semantic_route_description/get_route_region "{from_: 'ff_c3_begin', to: 'reima', personna: 'lambda'}"
```

The services definition is the following:
```
string from
string to
string personna
---
Route[] routes
float32[] costs
```

For the personna parameter, you have six types of persona that you can combine with the `_` delimiter:
 - lambda
 - disabled
 - knowing
 - notKnowing
 - young
 - old

The service will give you two arrays in return. Both arrays are the same size and give you a route (string array) and a cost for each corresponding route.


[License-Url]: https://opensource.org/licenses/Apache-2.0
[License-Image]: https://img.shields.io/badge/License-Apache%202.0-blue.svg
[Dependency-Image]: https://img.shields.io/badge/dependencies-ontoloGenius-1eb0fc.svg
[Dependency-Url]: https://github.com/sarthou/ontologenius
