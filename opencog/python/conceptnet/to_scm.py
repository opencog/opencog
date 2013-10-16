__author__ = 'Amen Belayneh'

# This code creates a scheme file after inputing the address to a conceptnet csv file dump
# The conceptnet csv file should be in the same folder as this script.
# Make sure you add .scm when inputting the name for the scheme output file
# The output file will be in the same folder as the script
 


from opencog.atomspace import AtomSpace, TruthValue,Link, Node
import reader
import term

corpus_path = ""
corpus_dict = {}
conceptnet_dict = {}
twf = 0

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
		if ("  "+ word.upper()) in  corpus_dict:
			stv.mean = float(corpus_dict[("  "+ word.upper())])/twf
			stv.count = .765 # have no reason for this value 
			conceptnet_dict[word] = stv
			return stv
		else:	
			stv.mean = 1/(twf + 1)
			stv.count = .043  # have no reason for this value
			conceptnet_dict[word] = stv
			return stv
		

def write_file(cn_assertion, context): # assertion is a list
	DEFAULT_TV = TruthValue(1,0.5) # 0.5 confidence is used(for links) because that is the weight given to most of the assertions on ConceptNet
	DEFAULT_TV2 = TruthValue()     # used for nodes
	
	if cn_assertion[0] == "/r/IsA":		
		return	('(ContextLink ' + '(stv '+ str(DEFAULT_TV.mean) +' ' +str(DEFAULT_TV.count) +')'  +  ' \n\t' +\
					'(ConceptNode  ' + '"' + str(context) + '"' + ' (stv {context_TV.mean} {context_TV.count})' + ')\n\t' +\
					'(InheritanceLink ' + '(stv '+ str(DEFAULT_TV.mean) +' ' +str(DEFAULT_TV.count) +')'  + '\n\t\t' +\
						'(ConceptNode  ' + '"' + cn_assertion[1][6:] + '"' + ' (stv {cn_assertion1.mean} {cn_assertion1.count})' + ')\n\t\t' +\
						'(ConceptNode  ' + '"' + cn_assertion[2][6:] + '"' + ' (stv {cn_assertion2.mean} {cn_assertion2.count})' + ')\n\t' +\
					')\n'	+\
				')').format(context_TV = set_TV(context), cn_assertion1 = set_TV(cn_assertion[1][6:]), cn_assertion2 = set_TV(cn_assertion[2][6:]) )
		
	else:
		return	('(ContextLink ' + '(stv '+ str(DEFAULT_TV.mean) +' ' +str(DEFAULT_TV.count)  +')' +  ' \n\t'  +\
					'(ConceptNode  ' + '"' + str(context) + '"' + ' (stv {context_TV.mean} {context_TV.count})' + ')\n\t' +\
					'(EvaluationLink  ' + '(stv '+ str(DEFAULT_TV.mean) +' ' +str(DEFAULT_TV.count) +')'  + '\n\t\t' +\
						'(PredicateNode  ' + '"' + cn_assertion[0][3:] + '"' +' ' + ' (stv '+ str(set_TV(cn_assertion[0][3:]).mean) +' ' +str(set_TV(cn_assertion[0][3:]).count)+ ')' + ')\n\t\t' +\
						'(ListLink  '	+ '\n\t\t\t' +\
								'(ConceptNode  ' + '"' + cn_assertion[1][6:] + '"' + ' (stv {cn_assertion1.mean} {cn_assertion1.count})' + ')\n\t\t\t' +\
								'(ConceptNode  ' + '"' + cn_assertion[2][6:] + '"' + ' (stv {cn_assertion2.mean} {cn_assertion2.count})' + ')\n\t\t' +\
						')\n\t'	 +\
					')\n' +\
				')').format(context_TV = set_TV(context), cn_assertion1 = set_TV(cn_assertion[1][6:]), cn_assertion2 = set_TV(cn_assertion[2][6:]) )


def from_file(cn_path, scm_name):
	lists_of_assertions = reader.csv(cn_path) # lists_of_assertions is a list of list of assertion
	with open(scm_name,'w') as scm_file:
		for an_assertion in lists_of_assertions:
			if an_assertion[3] == "/ctx/all":
				temp = write_file(an_assertion, "Universe")
				scm_file.write(temp+'\n'*2)
			
		else: # for other type of context naming is same as used in ConceptNet
			#map(an_assetion,an_assertion[3][5:],atomspace)
			pass	
	
			
if __name__ == '__main__':
	cn_url= raw_input("Enter ConceptNet csv file address: ")
	corpus_path = raw_input("Enter corpus address: ")
	name_of_scm_file = raw_input("Enter name for the Scheme Output file: ")
	from_file(cn_url,name_of_scm_file)
	print ("Scheme file is created successfully")
	
