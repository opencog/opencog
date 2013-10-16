__author__ = 'Amen Belayneh'

# This code reades word frequencies from a csv file, and
# calculates the term probability for each word by using this formula
# term_probability_of_a_word = word_frequency_of_the_word / total_word_frequency_of_all_the_words 


import csv


def read_frequencies(file_path, separator = ',', type_of_corpus = 0 ): # type_of_corpus values: TOTAL=0,SPOKEN=1,FICTION=2,MAGAZINE=3,NEWSPAPER=4,ACADEMIC=5
	container = [] # It is a list of lists.
	with open(file_path,'r') as stream:
		freq_list = csv.reader(stream,delimiter=separator) # a list of list of word frequencies
				
		for i in freq_list: 
			if i == '':
				break
			else:
				container.append([i[x] for x in [2,(type_of_corpus+3)]]) 
	
	del container[0]	# this is assuming the first row is a header
	return container
			

def total_freq(list_of_word_freq = []):
	total_word_freq = 0 
	for i in list_of_word_freq: 
		total_word_freq += float(i[1])
	return total_word_freq


if __name__ == "__main__":
	url= raw_input("Enter file address: ")
	print "###########################################"
	 
	while True:
		term =raw_input("Enter the term or q to end: ")
		if term == 'q': 
			print "be bye"
			break 
		corpus_type = int(raw_input ("Enter corpus type TOTAL=0,SPOKEN=1,FICTION=2,MAGAZINE=3,NEWSPAPER=4,ACADEMIC=5: "))
		corpus_list = read_frequencies(url,type_of_corpus = corpus_type)
		twf = total_freq(corpus_list)
		corpus_dict = dict(corpus_list)
		print "the term frequency and the total word frequency are ",str([corpus_dict["  "+ term.upper()], twf])
