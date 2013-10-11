__author__ = 'Amen Belayneh'

# This code reades word frequencies from a csv file, and
# calculates the term probability for each word by using this formula
# term_probability_of_a_word = word_frequency_of_the_word / total_word_frequency_of_all_the_words 


import csv


def read_frequencies(file_path,separator=',',type_of_corpus = 0 ): # type_of_corpus values: ALL=0,SPOKEN=1,FICTION=2,MAGAZINE=3,NEWSPAPER=4,ACADEMIC=5
	container = [] # container for edges in conceptnet
	with open(file_path,'r') as stream:
		freq_list = csv.reader(stream,delimiter=separator) # a list of list of word frequencies
				
		for i in freq_list: 
			#print i
			if i == '':
				break
			else:
				container.append([i[x] for x in [2,(type_of_corpus+3)]]) 
	
	del container[0]	# this is assuming the first row is a header
	
		
	return container	# container is a list of lists
	

def freq(term,corpus_path="",corpus_code = 0):
	if corpus_path == "":
		corpus_path= raw_input("Enter corpus address: ")
	
	list_of_freq = read_frequencies(corpus_path,type_of_corpus=corpus_code)
	
	total_word_freq = 0 # used to calculate the total_word_frequency_of_all_the_words 
	for i in list_of_freq: 
		total_word_freq += float(i[1])
				
	for i in list_of_freq: # returns the term probability if match is found in the corpus
		if i[0][2:].lower() == term.lower():
			print "Got the word"
			return [float(i[1]), total_word_freq]
		else:
			pass # Word is not in the corpus . Another corpus or method has to be used
		
	return [0, total_word_freq]
	


if __name__ == "__main__":
		
	url= raw_input("Enter corpus address: ")
	while True:
		term =raw_input("Enter the term or q to end: ")
			
		if term == 'q': 
			print "Bye!!!"
			break
		
		corpus_type = int(raw_input ("Enter corpus type to use - ALL=0,SPOKEN=1,FICTION=2,MAGAZINE=3,NEWSPAPER=4,ACADEMIC=5: "))
		temp = str(freq(term,url,corpus_code = corpus_type))
		
		print "Frequency of the word \'{}\' and the corpus total frequency is: {}".format(term,temp)
		
		print "************************************************\n"
