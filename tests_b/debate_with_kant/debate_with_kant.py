from tests_b.base_test_case import BaseTestCase
from util_b import blending_util
from util_b.blending_util import *
from util_b.general_util import BlConfig

__author__ = 'DongMin Kim'

from opencog.atomspace import types, TruthValue
from opencog.type_constructors \
    import ConceptNode, VariableNode, \
    MemberLink, InheritanceLink, AssociativeLink

# Debate with Kant example in the book 'The Way We Think'
# It is a mirror network example.
class DebateWithKantExample(BaseTestCase):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def __str__(self):
        return 'DebateWithKantExample'

    # Make all concept and link.
    def __make_atoms(self):
        #
        # Make nodes.
        #

        # Make default concepts.
        self.a_space = self.a.get_atoms_by_name(types.Atom, "Space")[0]
        self.a_frame = self.a.get_atoms_by_name(types.Atom, "Frame")[0]
        self.a_input_space_0 = ConceptNode("InputSpace0", self.default_atom_tv)
        self.a_input_space_1 = ConceptNode("InputSpace1", self.default_atom_tv)
        self.a_generic_space = ConceptNode("GenericSpace", self.default_atom_tv)
        self.a_blended_space = ConceptNode("BlendedSpace", self.default_atom_tv)

        make_link_all(
            self.a,
            types.InheritanceLink,
            [
                self.a_input_space_0,
                self.a_input_space_1,
                self.a_generic_space,
                self.a_blended_space
            ],
            self.a_space,
            self.default_link_tv
        )

        # Use in InputSpace0.
        self.a_kant = ConceptNode("Kant", rand_tv())
        self.a_claims = ConceptNode("Claims", rand_tv())
        self.a_musings = ConceptNode("Musings", rand_tv())
        self.a_writing = ConceptNode("Writing", rand_tv())
        self.a_german = ConceptNode("German", rand_tv())
        self.a_reason = ConceptNode("Reason", rand_tv())
        self.a_search_for_truth = ConceptNode("SearchForTruth", rand_tv())
        self.a_1784 = ConceptNode("1784", rand_tv())
        make_sti_all(
            self.a,
            [
                self.a_kant,
                self.a_claims,
                self.a_musings,
                self.a_writing,
                self.a_german,
                self.a_reason,
                self.a_search_for_truth,
                self.a_1784
            ],
            sti_value_dict['JUST_TARGET']
        )

        # Use in InputSpace1.
        self.a_me = ConceptNode("Me", rand_tv())
        self.a_speaking = ConceptNode("Speaking", rand_tv())
        self.a_english = ConceptNode("English", rand_tv())
        self.a_cognitive_process = ConceptNode("CognitiveProcess", rand_tv())
        self.a_1995 = ConceptNode("1995", rand_tv())
        self.a_dead = ConceptNode("Dead", rand_tv())
        self.a_aware = ConceptNode("Aware", rand_tv())
        make_sti_all(
            self.a,
            [
                self.a_me,
                self.a_speaking,
                self.a_english,
                self.a_cognitive_process,
                self.a_1995,
                self.a_dead,
                self.a_aware
            ],
            sti_value_dict['JUST_TARGET']
        )

        # Use in GenericSpace.
        self.a_thinker = ConceptNode("Thinker", rand_tv())
        self.a_mode_of_expression = ConceptNode("ModeOfExpression", rand_tv())
        self.a_language = ConceptNode("Language", rand_tv())
        self.a_issue = ConceptNode("Issue", rand_tv())
        self.a_purpose = ConceptNode("Purpose", rand_tv())
        self.a_time = ConceptNode("Time", rand_tv())
        # Not blend target.

        # Use in BlendedSpace.
        self.a_counterclaims = ConceptNode("Counterclaims", rand_tv())
        self.a_questions = ConceptNode("Questions", rand_tv())
        self.a_answers = ConceptNode("Answers", rand_tv())
        self.a_cognition = ConceptNode("Cognition", rand_tv())
        self.a_alive = ConceptNode("Alive", rand_tv())

        # Make 'Debate' frame.
        self.a_debate = ConceptNode("Debate", rand_tv())

        # Make 'Debate->Rhetorical Actions'
        self.a_rhetorical_actions = ConceptNode("RhetoricalActions", rand_tv())
        self.a_agrees = ConceptNode("Agrees", rand_tv())
        self.a_retorts = ConceptNode("Retorts", rand_tv())
        self.a_challenges = ConceptNode("Challenges", rand_tv())
        self.a_anticipates = ConceptNode("Anticipates", rand_tv())
        self.a_concurs = ConceptNode("Concurs", rand_tv())
        self.a_reinforces = ConceptNode("Reinforces", rand_tv())

        # Make 'Debate->Argumentation Connectives'
        self.a_argumentation_connectives = \
            ConceptNode("ArgumentationConnectives", rand_tv())
        self.a_but = ConceptNode("But", rand_tv())
        self.a_however = ConceptNode("However", rand_tv())
        self.a_therefore = ConceptNode("Therefore", rand_tv())
        self.a_on_the_contrary = ConceptNode("OnTheContrary", rand_tv())
        self.a_exactly = ConceptNode("Exactly", rand_tv())
        self.a_true_enough = ConceptNode("TrueEnough", rand_tv())
        self.a_not_so_fast = ConceptNode("NotSoFast", rand_tv())

        # Make 'Debate->AffirmationNegation'
        self.a_affirmation_negation = \
            ConceptNode("AffirmationNegation", rand_tv())
        self.a_yes = ConceptNode("Yes", rand_tv())
        self.a_no = ConceptNode("No", rand_tv())
        self.a_yes_or_no = ConceptNode("YesOrNo", rand_tv())

    # - Debate Frame
    def __make_debate_frame(self):
        # Make instance of frame concept.
        InheritanceLink(self.a_debate, self.a_frame, self.default_link_tv)

        # Link with debate frame type.
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.a_rhetorical_actions,
                self.a_argumentation_connectives,
                self.a_affirmation_negation
            ],
            self.a_debate,
            rand_tv()
        )

        # Link with several types.
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.a_agrees, self.a_retorts, self.a_challenges,
                self.a_anticipates, self.a_concurs, self.a_reinforces,
                self.a_questions, self.a_answers
            ],
            self.a_rhetorical_actions,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.a_but, self.a_however, self.a_therefore,
                self.a_on_the_contrary, self.a_exactly,
                self.a_true_enough, self.a_not_so_fast
            ],
            self.a_argumentation_connectives,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.a_yes, self.a_no, self.a_yes_or_no
            ],
            self.a_affirmation_negation,
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
                self.a_kant,
                self.a_claims,
                self.a_musings,
                self.a_writing,
                self.a_german,
                self.a_reason,
                self.a_search_for_truth,
                self.a_1784
            ],
            self.a_input_space_0
        )

        make_sti_all(
            self.a,
            [
                self.a_kant,
                self.a_claims,
                self.a_musings,
                self.a_writing,
                self.a_search_for_truth
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
                self.a_me,
                self.a_claims,
                self.a_musings,
                self.a_speaking,
                self.a_english,
                self.a_cognitive_process,
                self.a_search_for_truth,
                self.a_1995,
                self.a_kant,
                self.a_dead,
                self.a_aware
            ],
            self.a_input_space_1
        )

        # Make status of kant in modern.
        # TODO: Strong TruthValue instead of random value?
        MemberLink([self.a_kant, self.a_dead], rand_tv())

        # TODO: Is this expression is correct method?
        l_me_aware_kant = AssociativeLink([self.a_me, self.a_kant])
        l_kant_not_aware_me = AssociativeLink([self.a_kant, self.a_me])
        MemberLink([l_me_aware_kant, self.a_aware], TruthValue(0.9, 0.9))
        MemberLink([l_kant_not_aware_me, self.a_aware], TruthValue(0.1, 0.9))

        make_sti_all(
            self.a,
            [
                self.a_me,
                self.a_claims,
                self.a_musings,
                self.a_speaking,
                self.a_english,
                self.a_cognitive_process,
                self.a_search_for_truth,
                self.a_1995,
                self.a_kant,
                self.a_dead,
                self.a_aware
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
                self.a_thinker,
                self.a_claims,
                self.a_musings,
                self.a_mode_of_expression,
                self.a_language,
                self.a_issue,
                self.a_purpose,
                self.a_time
            ],
            self.a_generic_space,
            self.default_link_tv
        )

        # Link with several types.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.a_kant, self.a_me],
            self.a_thinker,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.a_writing, self.a_speaking],
            self.a_mode_of_expression,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.a_german, self.a_english],
            self.a_language,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.a_reason, self.a_cognitive_process],
            self.a_issue,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.a_search_for_truth],
            self.a_purpose,
            rand_tv()
        )
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.a_1784, self.a_1995],
            self.a_time,
            rand_tv()
        )

    # - Blended Space
    def __make_blended_space(self):
        pass

    def __link_with_blend_target_for_debug(self):
        # Link with blend target.
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.a_me,
                self.a_claims,
                self.a_musings,
                self.a_speaking,
                self.a_english,
                self.a_cognitive_process,
                self.a_search_for_truth,
                self.a_1995,
                self.a_kant,
                self.a_dead,
                self.a_aware
            ],
            self.a_blend_target,
            blend_target_link_tv
        )

        # Link with blend target.
        make_link_all(
            self.a,
            types.MemberLink,
            [
                self.a_kant,
                self.a_claims,
                self.a_musings,
                self.a_writing,
                self.a_german,
                self.a_reason,
                self.a_search_for_truth,
                self.a_1784
            ],
            self.a_blend_target,
            blend_target_link_tv
        )

    def make(self):
        self.__make_atoms()
        self.__make_debate_frame()
        self.__make_input_space_0()
        self.__make_input_space_1()
        self.__make_generic_space()
        self.__make_blended_space()

        if BlConfig().is_use_blend_target:
            self.__link_with_blend_target_for_debug()
