"""
Utility to transform an OpenCog subhypergraph into a directed graph and
return its representation in the DOT graph description language.

Refer to README.md before use.
"""

import uuid
from graphviz import Digraph
from opencog.atomspace import AtomSpace, TruthValue, types, get_type_name

__author__ = 'Cosmo Harrigan'

DEFAULT_DUPLICATED_TYPES = ['ConceptNode', 'PredicateNode']


def get_dot_representation(atomset, duplicated_types=DEFAULT_DUPLICATED_TYPES):
    """
    Transforms an OpenCog subhypergraph into a directed graph and returns its
    representation in the DOT graph description language.

    Parameters:

    atomset (required) The set of OpenCog atoms that defines the subhypergraph
    duplicated_types (optional, list of strings) The types that should be
       duplicated to reduce tangling in the graph due to a large amount of
       shared vertices. By default, it will duplicate ConceptNodes and
       PredicateNodes. A different set of types can optionally be passed as a
       parameter. You can also pass an empty list ([]) so that no types will
       be duplicated.

    Returns:

    A string, containing the DOT representation of the atomset.

    For more details on DOT, refer to:
       http://en.wikipedia.org/wiki/DOT_(graph_description_language)
       http://www.graphviz.org/Documentation.php

    Algorithm:

    1. Convert the hypergraph

      For each atom in the set to be processed:
        Let this atom be the source
        Create vertex
        Get outgoing set of atom
        For each item in outgoing set
          (If doesn't already exist as a vertex, create it)
          Let this atom be the target
          Create an edge from the source to the target

      resulting in a digraph.

    2. Remove tangling

      Create duplicate nodes for all nodes that are of a type included in
      duplicate_types

    3. Express the digraph in the DOT graph description language.
    """
    (vertices, edges) = process_atomset(atomset)

    (processed_vertices,
     processed_edges) = process_graph(vertices, edges, duplicated_types)

    #(dot_vertices,
    # dot_edges) = dot_from_graph(processed_vertices, processed_edges)

    #dot_output = make_dot_output(dot_vertices, dot_edges)
    dot_output = dot_from_graph(processed_vertices, processed_edges)

    return dot_output


def get_vertex(vertices, handle):
    """
    Finds a vertex identified by a specific identifier in the list of dicts
    """
    return next((item for item in vertices if item['handle'] == handle), None)


def make_duplicate_vertex(vertex):
    """
    Constructs a new vertex from a given vertex, with a unique identifier
    """
    new_vertex = {
        'handle': 'vertex_{0}'.format(uuid.uuid4().int),
        'name': vertex['name'],
        'type': vertex['type']
    }
    return new_vertex


def vertices_from_atomset(atomset):
    """
    Converts an atomset representing a subhypergraph into a set of vertices
    representing a graph
    """
    vertices = []
    for atom in atomset:
        vertex = {
            'type': get_type_name(atom.type),
            'name': atom.name,
            'handle': atom.h.value()
        }
        vertices.append(vertex)
    return vertices


def edges_from_atomset(atomset):
    """
    Finds the edges between vertices in the graph that is created by
    converting a subhypergraph into a set of vertices
    """
    edges = []
    for atom in atomset:
        for outgoing_atom in atom.out:
            edge = {
                'source': atom.h.value(),
                'target': outgoing_atom.h.value()
            }
            edges.append(edge)
    return edges


def process_atomset(atomset):
    """
    Returns the vertices and edges of a graph representation of a
    subhypergraph
    """
    return vertices_from_atomset(atomset), edges_from_atomset(atomset)


def process_graph(vertices, edges, duplicated_types):
    """
    Performs processing on a graph in order to apply simplification rules
    so that when the graph is rendered it will be easier to interpret

    Currently, these rules involve creating duplicate copies of certain
    vertex types.
    """
    processed_vertices = []
    processed_edges = []

    for edge in edges:
        processed_edge = {}

        source = get_vertex(vertices, edge['source'])

        # Check if this is a vertex type that is eligible for duplication
        if source['type'] in duplicated_types:

            # Make a duplicate copy of the source vertex
            new_vertex = make_duplicate_vertex(source)
            processed_edge['source'] = new_vertex['handle']
        else:

            # Otherwise, go ahead and use the original vertex
            processed_edge['source'] = source['handle']
            processed_vertices.append(source)

        target = get_vertex(vertices, edge['target'])

        # Check if this is a vertex type that is eligible for duplication
        if target['type'] in duplicated_types:

            # Make a duplicate copy of the target vertex
            new_vertex = make_duplicate_vertex(target)
            processed_edge['target'] = new_vertex['handle']
            processed_vertices.append(new_vertex)
        else:

            # Otherwise, go ahead and use the original vertex
            processed_edge['target'] = target['handle']
            processed_vertices.append(target)

        processed_edges.append(processed_edge)

    return processed_vertices, processed_edges


def dot_from_graph(processed_vertices, processed_edges):
    """
    Converts a set of vertices and edges from a graph into lines formatted
    using the DOT graph description language syntax
    """
    dot = Digraph(graph_attr={
        'rankdir': 'LR'
    }, comment='OpenCog Graph')

    for vertex in processed_vertices:
        make_dot_vertex(dot, vertex)

    for edge in processed_edges:
        make_dot_edge(dot, edge)

    return dot.source


def make_dot_vertex(dot, vertex):
    """
    Creates a vertex in DOT syntax. Includes a mapping of atom types to shapes.
    """
    if vertex['type'] == 'ConceptNode':
        shape = 'ellipse'
    elif vertex['type'] == 'PredicateNode':
        shape = 'octagon'
    elif vertex['type'] == 'ListLink':
        shape = 'component'
    elif vertex['type'] == 'EvaluationLink':
        shape = 'house'
    elif vertex['type'] == 'ImplicationLink':
        shape = 'tripleoctagon'
    elif vertex['type'] == 'VariableNode':
        shape = 'folder'
    else:
        shape = 'diamond'

    name = "[{0}] {1}".format(vertex['type'], vertex['name'])

    dot.node(name=str(vertex['handle']),
             label=name,
             shape=shape)


def make_dot_edge(dot, edge):
    """
    Creates an edge in DOT syntax.
    """
    dot.edge(str(edge['source']), str(edge['target']))
