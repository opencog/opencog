__author__ = 'keyvan'

from opencog.atomspace import AtomSpace, types, TruthValue
from random import randrange
from utility.generic import subsets_of_len_two

_default_tv = TruthValue(1,1)

class CombinationDescriptor(object):
    pass

class TypeDescriptor(CombinationDescriptor):

    def __init__(self, weight, object_type, block_type_descriptions=None):
        self.weight = weight
        self.object_type = object_type
        self.block_type_descriptions = block_type_descriptions

class SpatialRelationDescriptor(CombinationDescriptor):

    def __init__(self, weight, type_descriptors, relation):
        self.weight = weight
        self.type_descriptors = type_descriptors
        self.relation = relation

class SituationGenerator(object):

    def __init__(self, atomspace):
        self.atomspace = atomspace
        near_predicate_node = self.atomspace.add_node(types.PredicateNode, 'near', _default_tv)
        on_top_of_predicate_node = self.atomspace.add_node(types.PredicateNode, 'on-top-of', _default_tv)
        underneath_predicate_node = self.atomspace.add_node(types.PredicateNode, 'underneath', _default_tv)
        inside_predicate_node = self.atomspace.add_node(types.PredicateNode, 'inside', _default_tv)
        adjacent_to_predicate_node = self.atomspace.add_node(types.PredicateNode, 'adjacent-to', _default_tv)

        self.spatial_relations = [near_predicate_node, on_top_of_predicate_node,
                                  underneath_predicate_node, inside_predicate_node, adjacent_to_predicate_node]
        self._counter_by_type = {}

    def _generate_unique_name(self, object_type):

        if object_type in self._counter_by_type:
            self._counter_by_type[object_type] += 1
        else:
            self._counter_by_type[object_type] = 0

        return object_type + '_' +  str(self._counter_by_type[object_type])

    def _generate_block_entity_nodes_from_description(self, type_descriptor):
        nodes = []
        for i in xrange(type_descriptor.weight):
            entity_node = self.atomspace.add_node(types.BlockEntityNode,
                self._generate_unique_name(type_descriptor.object_type), _default_tv)
            if type_descriptor.block_type_descriptions is not None:
                for property, predicate_type, predicate_value in type_descriptor.block_type_descriptions:
                    for i in range(randrange(1,10)):
                        block = self.atomspace.add_node(types.StructureNode,
                            self._generate_unique_name('CHUNK_BLOCK'), _default_tv)
                        predicate_node = self.atomspace.add_node(types.PredicateNode, property, _default_tv)
                        value_node = self.atomspace.add_node(types.__dict__[predicate_type],
                            predicate_value, _default_tv)
                        list_link = self.atomspace.add_link(types.ListLink, [block, value_node])
                        self.atomspace.add_link(types.EvaluationLink, [predicate_node, list_link], _default_tv)

                        part_of_predicate = self.atomspace.add_node(types.PredicateNode, 'part-of', _default_tv)
                        list_link = self.atomspace.add_link(types.ListLink, [block, entity_node])
                        self.atomspace.add_link(types.EvaluationLink, [part_of_predicate, list_link], _default_tv)
            nodes.append(entity_node)
        return nodes

    def generate_situation(self, **kwargs):
        if 'randomness' not in kwargs:
            randomness=0
        else:
            randomness = kwargs['randomness']
        if 'type_descriptors' not in kwargs:
            type_descriptors = []
        else:
            type_descriptors = kwargs['type_descriptors']
        if 'spatial_relation_descriptors' not in kwargs:
            spatial_relation_descriptors = []
        else:
            spatial_relation_descriptors = kwargs['spatial_relation_descriptors']

        entity_nodes = []
        for descriptor in spatial_relation_descriptors:
            for i in range(descriptor.weight):
                for first_descriptor, second_descriptor in subsets_of_len_two(descriptor.type_descriptors):
                    for first_node in self._generate_block_entity_nodes_from_description(first_descriptor):
                        entity_nodes.append(first_node)
                        for second_node in self._generate_block_entity_nodes_from_description(second_descriptor):
                            entity_nodes.append(second_node)
                            list_link = self.atomspace.add_link(types.ListLink, [first_node, second_node])
                            predicate_node = self.atomspace.add_node(types.PredicateNode, descriptor.relation, _default_tv)
                            self.atomspace.add_link(types.EvaluationLink, [predicate_node, list_link], _default_tv)

        for descriptor in type_descriptors:
            entity_nodes += self._generate_block_entity_nodes_from_description(descriptor)

        number_of_entities = len(entity_nodes)
        for i in range(int(round(randomness * number_of_entities))):
            # select two random entities
            index_of_first_entity = randrange(number_of_entities)
            while True:
                index_of_second_entity = randrange(number_of_entities)
                if index_of_first_entity != index_of_second_entity:
                    break
            list_link = self.atomspace.add_link(types.ListLink,
                [entity_nodes[index_of_first_entity], entity_nodes[index_of_second_entity]])
            predicate_node =  self.spatial_relations[randrange(len(self.spatial_relations))]
            self.atomspace.add_link(types.EvaluationLink, [predicate_node, list_link], _default_tv)

def generate_sample_situation(atomspace):

    garden_descriptor = TypeDescriptor(10, 'tree', [('color', 'ConceptNode', 'green'),
                                                    ('color', 'ConceptNode', 'brown')])
    #    Describes 10 instances following Scheme definition of a tree:
    #
    #    (EvaluationLink (stv 1 0.0012484394)
    #        (PredicateNode "block-list")
    #        (ListLink
    #            (BlockEntityNode "tree" (av 1000 0 0))
    #            (ListLink
    #                (StructureNode "CHUNK_BLOCK_0")
    #                (StructureNode "CHUNK_BLOCK_1")
    #        )
    #    )
    #
    #    (EvaluationLink (stv 1 0.0012484394)
    #        (PredicateNode "color")
    #        (ListLink
    #            (StructureNode "CHUNK_BLOCK_0")
    #            (ConceptNode "green")
    #        )
    #    )
    #
    #    (EvaluationLink (stv 1 0.0012484394)
    #        (PredicateNode "color")
    #        (ListLink
    #            (StructureNode "CHUNK_BLOCK_1")
    #            (ConceptNode "brown")
    #        )
    #    )
    #
    #    Note: A random number >0 <10 of blocks with given description
    #    are generated, all bound to given 'block_type_descriptions' which
    #    is a list of tuples in form of:
    #    (property_name, predicate_type, predicate_value)
    #    e.g. for tree, block_type_descriptions would be:
    #    [('color', 'ConceptNode', 'green'), ('color', 'ConceptNode', 'brown')]


    house_descriptor = TypeDescriptor(1, 'house')

    village_descriptor = SpatialRelationDescriptor(3, [house_descriptor, garden_descriptor], 'adjacent-to')

    SituationGenerator(atomspace).generate_situation(spatial_relation_descriptors=[village_descriptor],randomness=0.2)

if __name__ == '__main__':
    atomspace = AtomSpace()
    generate_sample_situation(atomspace)
    atomspace.print_list()

#    print '\n==========================================='
#    print 'Fishgram'
#    print '===========================================\n'
#    from fishgram import *
#    fishAndChips = Fishgram(atomspace)
#    notice_changes(atomspace)
#    fishAndChips.forest.extractForest()
#    fishAndChips.run()