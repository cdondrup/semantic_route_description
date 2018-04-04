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
$ roslaunch semantic_route_description route_description.launch
```

## Call the route description

To get the route from **ff_c3_begin** to **reima**, call the getRoute service as follows:
```
$ rosservice call /semantic_route_description/getRoute "{from_: 'ff_c3_begin', to: 'reima'}"
```

The service definition is the following:
```
string from
string to
string personna
---
Route[] routes
float32[] costs
```

The personna parameter is not used at this time.
The service will give you two arrays in return. Both arrays are the same size and give you a route (string array) and a cost for each corresponding route.


[License-Url]: https://opensource.org/licenses/Apache-2.0
[License-Image]: https://img.shields.io/badge/License-Apache%202.0-blue.svg
[Dependency-Image]: https://img.shields.io/badge/dependencies-ontoloGenius-1eb0fc.svg
[Dependency-Url]: https://github.com/sarthou/ontologenius
