__author__ = 'Amen Belayneh'

# This code is used to read the relations from a coceptnet dump, and
# return a container containing a list of lists of the relations


def csv(csv_file_path):
    ''' Reads from csv dump of conceptnet.'''
    # container for edges in conceptnet
    # Each element of the container is of the format
    # [rel,start,end,context,weight],
    # the context and weight element are included for future.
    container = []
    with open(csv_file_path, 'r') as stream:
        line = []
        while line != '':
            line = stream.readline()
            if line == '':
                break
            else:
                temp = line.split('\t')
                container.append(temp[1:6])
    del container[0]
    return container   # container is a list of lists

if __name__ == "__main__":
    url = raw_input("Enter file address: ")
    container = csv(url)
    print (container)
