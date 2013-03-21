from xapian import InvalidOperationError

__author__ = 'Keyvan'

from opencog.atomspace import TruthValue
DEFAULT_TRUTH_VALUE = TruthValue(1,1)

class Marker(object): # Marker Class, usually used for marking!
    pass

class OtherThan(object):
    def __init__(self, seq):
        self.seq = seq
    def __eq__(self, other):
        for item in self.seq:
            if other == item:
                return False
        return True
    def __ne__(self, other):
        return not self.__eq__(other)

marker = Marker() # A Marker

def subsets_of_len_two(seq):
    indexed_seq = list(seq)
    length = len(indexed_seq)
    for i in xrange(length):
        for j in xrange(i + 1, length):
            yield (indexed_seq[i], indexed_seq[j])

def new_instance_of_same_type(parent):
    """
    returns an instance with
    the same type of the parent
    """
    instance = type(parent).__new__(type(parent))
    instance.__init__()
    return instance

def subsets_of(collection, subsets_type=set):
    subsets = lambda x: [subsets_type([y for j, y in enumerate(set(x)) if (i >> j) & 1])
                         for i in range(2**len(set(x)))]
    for subset in subsets(collection):
        yield subset

def dim(structure):
    return structure.__dim__()

import operator
def concat_lists(lists):
    return reduce(operator.concat, lists, [])

def read_scheme_data(scheme_file_path, atomspace):
    try:
        stream = open(scheme_file_path,'r')
    except:
        import urllib2
        stream = urllib2.urlopen(scheme_file_path)

    stack = []
    buffer = ''
    splitters = [' ', '(', ')']
    states = {
        'S0':{'(':('S1', None)},
        'S1':{'define':('Define', None), 'Node':('Node', None),'Link':('Link', None)},
        'Space':{' ':('Space', None)},
        'Node':{}

    }
    current_state = states['S0']

    lines = stream.readlines()
    for row, line in enumerate(lines):
        for column, char in enumerate(line):
            if char in ['\n',chr(9)]:
                continue
            if char in splitters:
                if buffer != '':
                    stack.append(buffer)
                    if buffer[-4:] in current_state:
                        next_state, action = current_state[buffer[-4]]
                        if action is not None:
                            action()
                        current_state = next_state
                    else:
                        raise InvalidOperationError('Parse error at ' + repr([row + 1, column + 1]))

                    buffer = ''
            else:
                buffer += char





    stream.close()

if __name__ == '__main__':
    print OtherThan([1,2]) != 2
#    from opencog.atomspace import AtomSpace
#    atomspace = AtomSpace()
#    read_scheme_data('https://dl.dropbox.com/s/wxjmg6etqsliot4/jade.scm?dl=1', atomspace)