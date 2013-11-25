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
map_dict = {"/r/IsA": 'InheritanceLink'}


def set_TV(word):
    global corpus_dict, conceptnet_dict, twf
    stv = TruthValue()
    if not(corpus_dict):
        term_lists = term.read_frequencies(corpus_path)
        corpus_dict = dict(term_lists)
        twf = term.total_freq(term_lists)

    try:
        stv = conceptnet_dict[word]
        return stv
    except KeyError:
        if ("  " + word.upper()) in corpus_dict:
            stv.mean = float(corpus_dict[("  " + word.upper())]) / twf
            stv.count = .95  # have no reason for this value
            conceptnet_dict[word] = stv
            return stv
        else:
            stv.mean = 1 / (twf + 1)
            stv.count = .95  # have no reason for this value
            conceptnet_dict[word] = stv
            return stv


def write_file(cn_assertion):
    # Assertion is a list. 0.5 confidence is used(for links)because that is
    # the weight given to most of the assertions on ConceptNet
    TV = TruthValue(1, 0.5)
    try:
        link_type = map_dict[cn_assertion[0]]
        return ('(' + str(link_type) + ' (stv ' + str(TV.mean) + ' ' + str(TV.count) + ')\n\t' +
                '(ConceptNode "' + cn_assertion[1][6:] + '" (stv {cn_argument1.mean} {cn_argument1.count}))\n\t' +
                '(ConceptNode "' + cn_assertion[2][6:] + '" (stv {cn_assertion2.mean} {cn_assertion2.count}))\n)'
                ).format(cn_argument1=set_TV(cn_assertion[1][6:]), cn_assertion2=set_TV(cn_assertion[2][6:]))
    except KeyError:
        return ('(EvaluationLink (stv ' + str(TV.mean) + ' ' + str(TV.count) + ')\n\t' +
                '(PredicateNode "' + cn_assertion[0][3:] + '" (stv ' + str(set_TV(cn_assertion[0][3:]).mean) + ' ' + str(set_TV(cn_assertion[0][3:]).count) + '))\n\t' +
                '(ListLink \n\t\t' +
                '(ConceptNode "' + cn_assertion[1][6:] + '" (stv {cn_argument1.mean} {cn_argument1.count}))\n\t\t' +
                '(ConceptNode "' + cn_assertion[2][6:] + '" (stv {cn_assertion2.mean} {cn_assertion2.count}))\n\t)\n)'
                ).format(cn_argument1=set_TV(cn_assertion[1][6:]), cn_assertion2=set_TV(cn_assertion[2][6:]))


def from_file(cn_path, scm_name):
    # lists_of_assertions is a list of list of assertion
    lists_of_assertions = reader.csv(cn_path)
    with open(scm_name, 'w') as scm_file:
        for an_assertion in lists_of_assertions:
            temp = write_file(an_assertion)
            scm_file.write(temp + '\n' * 2)

if __name__ == '__main__':
    cn_url = raw_input("Enter ConceptNet csv file address: ")
    corpus_path = raw_input("Enter corpus address: ")
    name_of_scm_file = raw_input("Enter name for the Scheme Output file: ")
    from_file(cn_url, name_of_scm_file)
    print ("Scheme file is created successfully")
