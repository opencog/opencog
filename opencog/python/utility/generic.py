__author__ = 'Keyvan'

class Marker(object): # Marker Class, usually used for marking!
    pass

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
    lines = stream.readlines()
    for line in lines:
        for char in line:
            if char in ['\n',chr(9)]:
                continue
            if char == '(':
                stack.append(char)
            elif char == ')':
                pass




    stream.close()

if __name__ == '__main__':
    from opencog.atomspace import AtomSpace
    atomspace = AtomSpace()
    read_scheme_data('https://dl.dropbox.com/s/wxjmg6etqsliot4/jade.scm?dl=1', atomspace)