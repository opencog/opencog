__author__ = 'Amen Belayneh'



def csv(csv_file_path):
	''' Reads from csv dump of conceptnet.'''
	stream = open(csv_file_path,'r')
	container = [] # container for all relations between objects
	line =[]
	
	
	while line != '':
		line = stream.readline()
		if line == '':
			break
		else:
			temp = line.split('\t')
			container.append(temp[1:6]) # each element of the container is of the format [rel,start,end,context,weight]
										# the context and weight element are included for future
	del container[0]
	return container

if __name__ == "__main__":
	url= raw_input("enter file address: ")
	container = csv(url)
	print (container)
	
