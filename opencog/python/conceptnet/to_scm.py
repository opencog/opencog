__author__ = 'Amen Belayneh'

# This code creates a scheme file, after inputing the address to a conceptnet
#  csv file dump. The conceptnet csv file should be in the same folder as this
# script. Make sure you add '.scm' when inputting the name for the scheme
# output file. The output file will be in the same folder as the script

from opencog.atomspace import TruthValue
import reader
import term

corpus_path = ""
corpus_dict = {}
conceptnet_dict = {}
twf = 0
# ConceptNet relation to Opencog Link mappings
map_dict = {"/r/ConceptuallyRelatedTo": "IntensionalSimilarityLink",
            "/r/ThematicKLine": "IntensionalSimilarityLink",
            "/r/SuperThematicKLine": "IntensionalInheritanceLink",
            "/r/IsA": "InheritanceLink",
            "/r/PropertyOf": "InheritanceLink",
            "/r/DefinedAs": "SimilarityLink",
            "/r/PrerequisiteEventOf": "RetroactiveImplicationLink",
            "/r/FirstSubeventOf": "StartsLink",
            "/r/SubeventOf": "DuringLink",
            "/r/LastSubeventOf": "EndsLink",
            "/r/EffectOf": "PredictiveImplicationLink"
            }


def set_TV(word):
    global corpus_dict, conceptnet_dict, twf
    if not(corpus_dict):
        term_lists = term.read_frequencies(corpus_path)
        corpus_dict = dict(term_lists)
        twf = term.total_freq(term_lists)

    try:
        stv = conceptnet_dict[word]
        return stv
    except KeyError:
        if ("  " + word.upper()) in corpus_dict:
            mean = float(corpus_dict[("  " + word.upper())]) / twf
            count = .95  # have no reason for this value
            conceptnet_dict[word] = TruthValue(mean, count)
            return conceptnet_dict[word]
        else:
            mean = 1 / (twf + 1)
            count = .95  # have no reason for this value
            conceptnet_dict[word] = TruthValue(mean, count)
            return conceptnet_dict[word]


def print_TV(word, case=1):
    return('(stv ' + str(word.mean) + ' ' + str(word.count) + ')')


def write_atoms(cn_assertion):
    # Assertion is a list. 0.5 confidence is used(for links)because that is
    # the weight given to most of the assertions on ConceptNet
    TV = TruthValue(1, .5)
    try:
        link_type = map_dict[cn_assertion[0]]
        return ('(' + str(link_type) + ' {link_tv}\n\t' +
                '(ConceptNode "{cn_argument1}" {cn_arg1_stv})\n\t' +
                '(ConceptNode "{cn_argument2}" {cn_arg2_stv})\n'
                ).format(link_tv=print_TV(TV),
                         cn_argument1=cn_assertion[1][6:],
                         cn_arg1_stv=print_TV(set_TV(cn_assertion[1][6:])),
                         cn_argument2=cn_assertion[2][6:],
                         cn_arg2_stv=print_TV(set_TV(cn_assertion[2][6:])))
    except KeyError:
        return ('(EvaluationLink' + ' {link_tv}\n\t' +
                '(PredicateNode "{cn_relation}" {cn_rel_stv})\n\t' +
                '(ListLink\n\t\t' +
                '(ConceptNode "{cn_argument1}" {cn_arg1_stv})\n\t\t' +
                '(ConceptNode "{cn_argument2}" {cn_arg2_stv})\n\t)\n)'
                ).format(link_tv=print_TV(TV),
                         cn_relation=cn_assertion[0][3:],
                         cn_rel_stv=print_TV(set_TV(cn_assertion[0][3:])),
                         cn_argument1=cn_assertion[1][6:],
                         cn_arg1_stv=print_TV(set_TV(cn_assertion[1][6:])),
                         cn_argument2=cn_assertion[2][6:],
                         cn_arg2_stv=print_TV(set_TV(cn_assertion[2][6:])))


def from_file(cn_path, scm_name):
    # lists_of_assertions is a list of list of assertion
    lists_of_assertions = reader.csv(cn_path)
    with open(scm_name, 'w') as scm_file:
        for an_assertion in lists_of_assertions:
            temp = write_atoms(an_assertion)
            scm_file.write(temp + '\n' * 2)

if __name__ == '__main__':
    cn_url = raw_input("Enter ConceptNet csv file address: ")
    corpus_path = raw_input("Enter corpus address: ")
    name_of_scm_file = raw_input("Enter name for the Scheme Output file: ")
    from_file(cn_url, name_of_scm_file)
    print ("Scheme file is created successfully")
