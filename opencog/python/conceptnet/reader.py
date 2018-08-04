import numpy as np

__author__ = 'Amen Belayneh'

# This code is used to read the relations from a ConceptNet dump, and
# read the word frequencies from a csv file,
# Returns a container containing a list of lists of the relations
# Returns


class ConceptNetReader:
    def __init__(self, cn_path, cp_path):
        self.concept_net_file_path = cn_path
        self.corpus_file_path = cp_path

    def ascii_lines(self, iterable):
        for line in iterable:
            if all(ord(ch) < 128 and ch not in "\\\"`" for ch in line):
                continue
            else:
                return False
        return True

    def read_corpus_file(self, type_of_corpus=0, separator=','):
        # type_of_corpus values:
        # TOTAL=0,SPOKEN=1,FICTION=2,MAGAZINE=3,NEWSPAPER=4,ACADEMIC=5
        return np.genfromtxt(
            self.corpus_file_path,
            comments=None,
            dtype=np.dtype(str),
            usecols=(2, type_of_corpus + 3),
            delimiter=separator,
            skip_header=1
        ).tolist()

    def read_concept_net_file(self, start_row=0):
        """ Reads from csv dump of ConceptNet."""
        # container for edges in ConceptNet
        # Each element of the container is of the format
        # [rel,start,end,context,weight],
        # the context and weight element are included for future.

        container_np_array = np.genfromtxt(
            self.concept_net_file_path,
            comments=None,
            dtype=np.dtype(str),
            usecols=range(1, 6),
            delimiter='\t',
            skip_header=start_row
        )

        container_list = set(filter(
            lambda x: x[1].startswith('/c/en') and self.ascii_lines(x),
            tuple(map(tuple, container_np_array))
        ))

        return container_list   # container_list is a list of lists

if __name__ == "__main__":
    cn_path = raw_input("Enter ConceptNet file address: ")
    cp_path = raw_input("Enter corpus file address: ")
    corpus_type = int(raw_input(
        "Enter corpus type TOTAL=0,SPOKEN=1,FICTION=2,MAGAZINE=3," +
        "NEWSPAPER=4,ACADEMIC=5: "))
    reader = ConceptNetReader(cn_path, cp_path)
    print reader.read_concept_net_file()
    print dict(reader.read_corpus_file(corpus_type))
