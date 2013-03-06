__author__ = 'keyvan&ramin'

class Quad(object):
    def __init__(self, subject, predicate, destination, value):
        self.subject = subject
        self.predicate = predicate
#        if 'destination' not in kwargs and 'value' not in kwargs:
#            raise KeyError('Either value or destination should be provided.')
#        self.__dict__.update(kwargs)
        self.destination = destination
        self.value = value

    def __repr__(self):
        return repr([self.subject, self.predicate, self.destination, self.value])

def extract_quads(quad_file_path):
    try:
        stream = open(quad_file_path,'r')
    except:
        import urllib2
        stream = urllib2.urlopen(quad_file_path)

    quads = []
    lines = stream.readlines()
    for line in lines:
        splitted_string = (line.strip("\t\n")).split("\t")
        if len(splitted_string) > 3:
            quads.append(Quad(splitted_string[0],splitted_string[1],
                None if splitted_string[2] == '' else splitted_string[2],splitted_string[3]))
        else:
            quads.append(Quad(splitted_string[0],splitted_string[1],splitted_string[2],None))


    return quads

if __name__ == "__main__":
    for quad in extract_quads('http://wiki.freebase.com/images/e/eb/Steve-martin-quad-sample.txt'):
        print quad