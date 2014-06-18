DOT Graph Description Language Utility
======================================

Transforms an OpenCog subhypergraph into a directed graph and returns its
representation in the DOT textual graph description language.

### Prerequisites

**Requires Graphviz**

```
apt-get install graphviz
```

Or, for the newest version, use:

http://www.graphviz.org/Download_linux_ubuntu.php

**Requires the Python graphviz bindings**

```
pip install graphviz
```

https://pypi.python.org/pypi/graphviz

### Example

```
import dot
dot_output = dot.get_dot_representation(atomset)
```

See the full usage example in **example.py**.

### Example Visual Output
![Example](https://raw.githubusercontent.com/cosmoharrigan/diagrams/master/smokes/png/duplicate-conceptnode-predicatenode-shapes.png)

### Parameters

- ```atomset``` ***(required)*** The set of OpenCog atoms that defines the subhypergraph

- ```duplicated_types``` ***(optional, list of strings)***
    The types that should be duplicated to reduce tangling in the graph due to a large amount of shared vertices.
    By default, it will duplicate ConceptNodes and PredicateNodes.
    A different set of types can optionally be passed as a parameter.
    You can also pass an empty list ([]) so that no types will be duplicated.

### Returns

A string, containing the DOT representation of the atomset.

#### Example

```
digraph OpenCog {
rankdir=LR
1 [label="[ConceptNode] Dog" shape=ellipse]
}
```

### The DOT Graph Description Language

For more details on DOT, refer to:

http://en.wikipedia.org/wiki/DOT_(graph_description_language)

http://www.graphviz.org/Documentation.php

### Algorithm

#### Convert the hypergraph into a graph

For each atom in the set to be processed:

1. Let this atom be the source
2. Create vertex
3. Get outgoing set of atom
4. For each item in outgoing set:
    - Let this atom be the target
    - Create an edge from the source to the target

resulting in a digraph.

#### Remove tangling

Create duplicate nodes for all nodes that are of a type included in **duplicated_types**

#### Create DOT format
Express the digraph in the DOT graph description language.

### Visualizing the output
Use Graphviz to render the output for visualization. See the example in **example-visualization.py**.
