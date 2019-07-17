__author__ = 'Amen Belayneh'

# This code creates a scheme or python file, after inputting the address to
# a ConceptNet csv file dump.
# The ConceptNet csv file should be in the same folder as this script.
# The output file will be in the same folder as the script.

import sys
import math
from opencog.atomspace import TruthValue, types, get_type, AtomSpace
from reader import ConceptNetReader
from writer import ConceptNetWriter


class ConceptNetConverter:
    def __init__(self, a, cn_path, out_type, out_name, cp_path, start_row):
        self.a = a
        self.concept_net_start_row = start_row

        self.corpus_dict = None
        self.concept_net_dict = dict()
        self.twf = 0

        self.reader_test = ConceptNetReader(cn_path, cp_path)
        self.writer_test = ConceptNetWriter(a, out_type, out_name)

        # Make a ConceptNet relation to Opencog Link map
        """
        Temporary changed due to broken spacetime link types:
        "/r/HasSubevent" was "DuringLink",
        "/r/FirstSubeventOf" was "StartsLink",
        "/r/SubeventOf" was "DuringLink",
        "/r/LastSubeventOf" was "EndsLink",
        "/r/EffectOf" was "PredictiveImplicationLink",
        "/r/Causes" was "PredictiveImplicationLink"
        "/r/HasPrerequisite" was "RetroactiveImplicationLink",
        "/r/PrerequisiteEventOf" was "RetroactiveImplicationLink",
        """

        self.map_dict = \
            {
                "/r/ConceptuallyRelatedTo": "IntensionalSimilarityLink",
                "/r/ThematicKLine": "IntensionalSimilarityLink",
                "/r/SuperThematicKLine": "IntensionalInheritanceLink",

                "/r/HasProperty": "IntensionalInheritanceLink",
                "/r/HasSubevent": "EvaluationLink",

                "/r/IsA": "InheritanceLink",
                "/r/PropertyOf": "InheritanceLink",

                "/r/DefinedAs": "SimilarityLink",

                "/r/HasPrerequisite": "EvaluationLink",
                "/r/PrerequisiteEventOf": "EvaluationLink",

                "/r/FirstSubeventOf": "EvaluationLink",
                "/r/SubeventOf": "EvaluationLink",
                "/r/LastSubeventOf": "EvaluationLink",

                "/r/EffectOf": "EvaluationLink",
                "/r/Causes": "EvaluationLink"
            }

        if types is None:
            raise EnvironmentError("Please import AtomSpace lib first.")

    # 1. ConceptNet relation to Opencog Link mappings
    def to_link_type(self, relation):
        return self.map_dict[relation] if relation in self.map_dict \
            else "EvaluationLink"

    def is_effect_of(self, relation):
        return "EvaluationLink" if relation == "/r/EffectOf" else False
    # end of 1.

    # 2. Calculates the probability and makes TruthValue
    def total_freq(self, list_of_word_freq):
        # Calculates the term probability for each word by using this formula
        # term_probability_of_a_word =
        # word_frequency_of_the_word / total_word_frequency_of_all_the_words
        total_word_freq = 0
        if type(list_of_word_freq) is not list:
            return total_word_freq

        for i in list_of_word_freq:
            total_word_freq += float(i[1])
        return total_word_freq

    def make_opencog_tv(self, word):
        if word in self.concept_net_dict:
            return self.concept_net_dict[word]

        if ("  " + word.upper()) in self.corpus_dict:
            mean = float(self.corpus_dict[("  " + word.upper())]) / self.twf
            count = .95  # have no reason for this value
            self.concept_net_dict[word] = TruthValue(mean, count)
            return self.concept_net_dict[word]
        else:
            mean = 1 / (self.twf + 1)
            count = .95  # have no reason for this value
            self.concept_net_dict[word] = TruthValue(mean, count)
            return self.concept_net_dict[word]
    # end of 2.

    # 3. Makes OpenCog nodes and links.
    def __make_atoms(self, assertion, is_eval_link, link_type=''):
        # /c/en/computer -> computer
        node_1_name = assertion[1][6:]
        node_2_name = assertion[2][6:]

        node_1_stv = self.make_opencog_tv(node_1_name)
        node_2_stv = self.make_opencog_tv(node_2_name)

        link_confidence = float(assertion[4])
        if link_confidence > 0.0:
            # For true statements we give a strength of 0.5 to 1.0, ramping up
            # quickly towards 1.0 as the cn_confidence grows.
            confidence_value = 1.0 - math.pow(0.2, link_confidence)/2.0
        else:
            # For any false statement we currently just set a strength of 0.0,
            # basically ignoring the ConceptNet confidence value other than
            # to note that it was false.
            # TODO: This could probably be improved, but not sure how.
            confidence_value = 0.0

        # For the STV we use the computed confidence from ConceptNet as the
        # 'strength'. Currently just using a 'confidence' of ~0.9 indicating
        # that whatever ConceptNet thinks the truth value is, we agree strongly
        # with this analysis. Maybe something better could be done, hard to say
        # for sure though. The 10000 gets converted to the confidence of ~0.9
        # by the constructor.
        link_tv = TruthValue(confidence_value, 10000)

        node_1 = self.a.add_node(types.ConceptNode, node_1_name, tv=node_1_stv)
        node_2 = self.a.add_node(types.ConceptNode, node_2_name, tv=node_2_stv)

        if is_eval_link:
            relation = assertion[0][3:]
            predicate_node = self.a.add_node(
                types.PredicateNode,
                relation,
                tv=link_tv
            )
            list_link = self.a.add_link(types.ListLink, [node_1, node_2])
            eval_link = self.a.add_link(
                types.EvaluationLink,
                [predicate_node, list_link],
                tv=link_tv
            )
            return eval_link
        else:
            link = self.a.add_link(
                get_type(link_type),
                [node_1, node_2],
                tv=link_tv
            )
            return link

    def make_eval_link(self, assertion, link_type=''):
        return self.__make_atoms(assertion, True, link_type)

    def make_direct_link(self, assertion, link_type=''):
        return self.__make_atoms(assertion, False, link_type)
    # end of 3.

    # 4. Main routine
    def convert(self):
        # lists_of_assertions is a list of list of assertion
        lists_of_assertions = \
            self.reader_test.read_concept_net_file(self.concept_net_start_row)

        # Make a corpus dict
        term_lists = self.reader_test.read_corpus_file()
        self.corpus_dict = dict(term_lists)
        self.twf = self.total_freq(term_lists)

        for assertion in lists_of_assertions:
            relation = assertion[0]
            if self.is_effect_of(relation):
                output_atom = self.make_eval_link(assertion)
                self.writer_test.write_to_file(output_atom)

            if (self.to_link_type(relation) == "EvaluationLink") and \
                    (self.is_effect_of(relation) != "EvaluationLink"):
                # this condition is to prevent repetition of EvaluationLink
                output_atom = self.make_eval_link(assertion)
                self.writer_test.write_to_file(output_atom)
            elif self.to_link_type(relation) != "EvaluationLink":
                output_atom = self.make_direct_link(
                    assertion, self.to_link_type(relation)
                )
                self.writer_test.write_to_file(output_atom)
    # end of 4.

if __name__ == '__main__':
    # If the script was called with no command line arguments
    use_predefined_parameter = False
    concept_net_file_path = "conceptnet4.csv"
    corpus_file_path = "60000_Word_Freqs.csv"
    output_file_type = "scm"
    output_file_name = "conceptnet4.scm"
    concept_net_start_row = 0

    if use_predefined_parameter:
        pass
    elif len(sys.argv) == 1:
        concept_net_file_path = raw_input("Enter ConceptNet csv file address: ")
        corpus_file_path = raw_input("Enter corpus address: ")
        output_file_type = raw_input("Enter type for the Output(scm or py): ")
        output_file_name = raw_input("Enter name for the Output file: ")
        concept_net_start_row = raw_input(
            "Enter the position of start row in ConceptNet(Optional):"
        )

    if 1 < len(sys.argv) < 5:
        print "\n\n\tUsage: " + sys.argv[0] + \
              " input.csv corpus_freqs.csv scm output_file.scm 0\n\n"
        sys.exit(1)

    if len(sys.argv) >= 5:
        concept_net_file_path = sys.argv[1]
        corpus_file_path = sys.argv[2]
        output_file_type = sys.argv[3]
        output_file_name = sys.argv[4]

    if len(sys.argv) >= 6:
        concept_net_start_row = sys.argv[5]

    try:
        concept_net_start_row = int(concept_net_start_row)
    except ValueError:
        concept_net_start_row = 0

    ConceptNetConverter(
        AtomSpace(),
        concept_net_file_path,
        output_file_type,
        output_file_name,
        corpus_file_path,
        concept_net_start_row
    ).convert()
    print ("Converting was successfully finished.")
