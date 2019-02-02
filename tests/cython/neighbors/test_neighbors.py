from unittest import TestCase

from opencog.atomspace import AtomSpace, TruthValue, Atom
from opencog.atomspace import types, is_a, get_type, get_type_name

from opencog.type_constructors import *
from opencog.utilities import initialize_opencog, finalize_opencog

from time import sleep

class NeighborTest(TestCase):

    def setUp(self):
        self.space = AtomSpace()
        initialize_opencog(self.space)

    def tearDown(self):
        finalize_opencog()
        del self.space

    def test_get_predicates(self):
        dog = ConceptNode("dog")
        mammal = ConceptNode("mammal")
        canine = ConceptNode("canine")
        animal = ConceptNode("animal")
        dog_mammal = ListLink(dog, mammal)
        dog_canine = ListLink(dog, canine)
        dog_animal = ListLink(dog, animal)
        isA = PredicateNode("IsA")
        dogIsAMammal = EvaluationLink(isA, dog_mammal)
        dogIsACanine = EvaluationLink(isA, dog_canine)
        dogIsAAnimal = EvaluationLink(isA, dog_animal)

        dog_predicates = self.space.get_predicates(dog)
        self.assertEquals(len(dog_predicates), 3)

        count = 0
        for dogIs in self.space.get_predicates(dog):
            count += 1
        self.assertEquals(count, 3)

    def test_get_predicates_for(self):
        dog = ConceptNode("dog")
        mammal = ConceptNode("mammal")
        canine = ConceptNode("canine")
        animal = ConceptNode("animal")
        dog_mammal = ListLink(dog, mammal)
        dog_canine = ListLink(dog, canine)
        dog_animal = ListLink(dog, animal)
        isA = PredicateNode("IsA")
        dogIsAMammal = EvaluationLink(isA, dog_mammal)
        dogIsACanine = EvaluationLink(isA, dog_canine)
        dogIsAAnimal = EvaluationLink(isA, dog_animal)

        human = ConceptNode("human")
        dog_human = ListLink(dog, human)
        loves = PredicateNode("loves")
        dogLovesHumans = EvaluationLink(loves, dog_human)

        dog_predicates = self.space.get_predicates_for(dog, isA)
        self.assertEquals(len(dog_predicates), 3)

        dog_predicates = self.space.get_predicates_for(dog, loves)
        self.assertEquals(len(dog_predicates), 1)

        count = 0
        for dogIsA in self.space.get_predicates_for(dog, isA):
            count += 1
        self.assertEquals(count, 3)

        count = 0
        for dogLoves in self.space.get_predicates_for(dog, loves):
            count += 1
        self.assertEquals(count, 1)
