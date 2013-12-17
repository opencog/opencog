__author__ = 'Amen Belayneh'

# This code is used to read the relations from a conceptnet dump, and
# return a container containing a list of lists of the relations


def csv(csv_file_path):
    ''' Reads from csv dump of conceptnet.'''
    # container for edges in conceptnet
    # Each element of the container is of the format
    # [rel,start,end,context,weight],
    # the context and weight element are included for future.

    # You have to open it in utf8 encoding because the conceptnet CSV file uses utf8.
    container = []
    import codecs
    with codecs.open(csv_file_path, 'r', encoding='utf-8') as stream:
        line = []
        while line != '':
            line = stream.readline()
            if line == '':
                break
            else:
                # convert it to ascii (required for atomspace)
                line = line.encode('ascii','xmlcharrefreplace')
                temp = line.split('\t')
                if temp[1:6] not in container:
                    container.append(temp[1:6])
    del container[0]
    return container   # container is a list of lists

if __name__ == "__main__":
    url = raw_input("Enter file address: ")
    container = csv(url)
    print (container)
