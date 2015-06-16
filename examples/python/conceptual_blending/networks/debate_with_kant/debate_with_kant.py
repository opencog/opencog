# coding=utf-8
from examples.python.conceptual_blending.networks.base_network import *
from opencog_b.python.conceptual_blending.util.blending_util import *
from opencog.type_constructors import *

__author__ = 'DongMin Kim'


# Debate with Kant example in the book 'The Way We Think'
# It is a mirror network example.
class DebateWithKantNetwork(BaseNetwork):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def __str__(self):
        return self.__class__.__name__

    # Make all concept and link.
    def __make_atoms(self):
        #
        # Make nodes.
        #

        # Make default concepts.
        self.space = ConceptNode("Space", self.default_atom_tv)
        self.frame = ConceptNode("Frame", self.default_atom_tv)
        self.input_space_0 = ConceptNode("InputSpace0", self.default_atom_tv)
        self.input_space_1 = ConceptNode("InputSpace1", self.default_atom_tv)
        self.generic_space = ConceptNode("GenericSpace", self.default_atom_tv)
        self.blend_space = ConceptNode("BlendSpace", self.default_atom_tv)

        make_link_all(
            self.a,
            types.InheritanceLink,
            [
                self.input_space_0,
                self.input_space_1,
                self.generic_space,
                self.blend_space
            ],
            self.space,
            self.default_link_tv
        )

        # Use in InputSpace0.
        self.kant = ConceptNode("Kant", rand_tv())
        self.claims = ConceptNode("Claims", rand_tv())
        self.musings = ConceptNode("Musings", rand_tv())
        self.writing = ConceptNode("Writing", rand_tv())
        self.german = ConceptNode("German", rand_tv())
        self.reason = ConceptNode("Reason", rand_tv())
        self.search_for_truth = ConceptNode("SearchForTruth", rand_tv())
        self.a_1784 = ConceptNode("1784", rand_tv())
        make_sti_all(
            [
                self.kant,
                self.claims,
                self.musings,
                self.writing,
                self.german,
                self.reason,
                self.search_for_truth,
                self.a_1784
            ],
            sti_value_dict['JUST_TARGET']
        )

        # Use in InputSpace1.
        self.me = ConceptNode("Me", rand_tv())
        self.speaking = ConceptNode("Speaking", rand_tv())
        self.english = ConceptNode("English", rand_tv())
        self.cognitive_process = ConceptNode("CognitiveProcess", rand_tv())
        self.a_1995 = ConceptNode("1995", rand_tv())
        self.dead = ConceptNode("Dead", rand_tv())
        self.aware = ConceptNode("Aware", rand_tv())
        make_sti_all(
            [
                self.me,
                self.speaking,
                self.english,
                self.cognitive_process,
                self.a_1995,
                self.dead,
                self.aware
            ],
            sti_value_dict['JUST_TARGET']
        )

        # Use in GenericSpace.
        self.thinker = ConceptNode("Thinker", rand_tv())
        self.mode_of_expression = ConceptNode("ModeOfExpression", rand_tv())
        self.language = ConceptNode("Language", rand_tv())
        self.issue = ConceptNode("Issue", rand_tv())
        self.purpose = ConceptNode("Purpose", rand_tv())
        self.time = ConceptNode("Time", rand_tv())

        # Use in BlendSpace.
        self.counterclaims = ConceptNode("Counterclaims", rand_tv())
        self.questions = ConceptNode("Questions", rand_tv())
        self.answers = ConceptNode("Answers", rand_tv())
        self.cognition = ConceptNode("Cognition", rand_tv())
        self.alive = ConceptNode("Alive", rand_tv())

        # Make 'Debate' frame.
        self.debate = ConceptNode("Debate", rand_tv())

        # Make 'Debate->Rhetorical Actions'
        self.rhetorical_actions = ConceptNode("RhetoricalActions", rand_tv())
        self.agrees = ConceptNode("Agrees", rand_tv())
        self.retorts = ConceptNode("Retorts", rand_tv())
        self.challenges = ConceptNode("Challenges", rand_tv())
        self.anticipates = ConceptNode("Anticipates", rand_tv())
        self.concurs = ConceptNode("Concurs", rand_tv())
        self.reinforces = ConceptNode("Reinforces", rand_tv())

        # Make 'Debate->Argumentation Connectives'
        self.argumentation_connectives = \
            ConceptNode("ArgumentationConnectives", rand_tv())
        self.but = ConceptNode("But", rand_tv())
        self.however = ConceptNode("However", rand_tv())
        self.therefore = ConceptNode("Therefore", rand_tv())
        self.on_the_contrary = ConceptNode("OnTheContrary", rand_tv())
        self.exactly = ConceptNode("Exactly", rand_tv())
        self.true_enough = ConceptNode("TrueEnough", rand_tv())
        self.not_so_fast = ConceptNode("NotSoFast", rand_tv())

        # Make 'Debate->AffirmationNegation'
        self.affirmation_negation = \
            ConceptNode("AffirmationNegation", rand_tv())
        self.yes = ConceptNode("Yes", rand_tv())
        self.no = ConceptNode("No", rand_tv())
        self.yes_or_no = ConceptNode("YesOrNo", rand_tv())

    # - Debate Frame
    def __make_debate_frame(self):
        # Make instance of frame concept.
        InheritanceLink(self.debate, self.frame, self.default_link_tv)

        # Link with debate frame type.
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.rhetorical_actions,
                self.argumentation_connectives,
                self.affirmation_negation
            ],
            self.debate,
            rand_tv()
        )

        # Link with several types.
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.agrees, self.retorts, self.challenges,
                self.anticipates, self.concurs, self.reinforces,
                self.questions, self.answers
            ],
            self.rhetorical_actions,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.but, self.however, self.therefore,
                self.on_the_contrary, self.exactly,
                self.true_enough, self.not_so_fast
            ],
            self.argumentation_connectives,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.yes, self.no, self.yes_or_no
            ],
            self.affirmation_negation,
            rand_tv()
        )

    # - Input Space 0
    # Start to make 'Original Kant' network.
    def __make_input_space_0(self):
        # Make instance of space concept.
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.kant,
                self.claims,
                self.musings,
                self.writing,
                self.german,
                self.reason,
                self.search_for_truth,
                self.a_1784
            ],
            self.input_space_0
        )

        make_sti_all(
            [
                self.kant,
                self.claims,
                self.musings,
                self.writing,
                self.search_for_truth
            ],
            sti_value_dict['IMPORTANT']
        )

    # - Input Space 1
    # Start to make 'Modern Philosopher' network.
    def __make_input_space_1(self):
        # Make instance of space concept.
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.me,
                self.claims,
                self.musings,
                self.speaking,
                self.english,
                self.cognitive_process,
                self.search_for_truth,
                self.a_1995,
                self.kant,
                self.dead,
                self.aware
            ],
            self.input_space_1
        )

        # Make status of kant in modern.
        # TODO: Strong TruthValue instead of random value?
        # 죽었다는게 확실하니 확실한 진릿값을 주어야 하나?
        MemberLink([self.kant, self.dead], rand_tv())

        # TODO: Is this expression is correct method?
        # 겹치는 이 표현방법이 맞는 방법인가?
        l_me_aware_kant = AssociativeLink([self.me, self.kant])
        l_kant_not_aware_me = AssociativeLink([self.kant, self.me])
        MemberLink([l_me_aware_kant, self.aware], TruthValue(0.9, 0.9))
        MemberLink([l_kant_not_aware_me, self.aware], TruthValue(0.1, 0.9))

        make_sti_all(
            [
                self.me,
                self.claims,
                self.musings,
                self.speaking,
                self.english,
                self.cognitive_process,
                self.search_for_truth,
                self.a_1995,
                self.kant,
                self.dead,
                self.aware
            ],
            sti_value_dict['IMPORTANT']
        )

    # - Generic Space
    def __make_generic_space(self):
        # Make instance of space concept.
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.thinker,
                self.claims,
                self.musings,
                self.mode_of_expression,
                self.language,
                self.issue,
                self.purpose,
                self.time
            ],
            self.generic_space,
            self.default_link_tv
        )

        # Link with several types.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.kant, self.me],
            self.thinker,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.writing, self.speaking],
            self.mode_of_expression,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.german, self.english],
            self.language,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.reason, self.cognitive_process],
            self.issue,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.search_for_truth],
            self.purpose,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.a_1784, self.a_1995],
            self.time,
            rand_tv()
        )

    # - Blended Space
    def __make_blend_space(self):
        pass

    def make(self):
        self.__make_atoms()
        self.__make_debate_frame()
        self.__make_input_space_0()
        self.__make_input_space_1()
        self.__make_generic_space()
        self.__make_blend_space()
