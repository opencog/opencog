__author__ = 'Amen Belayneh'

# This code creates a scheme file after inputing the address to a conceptnet csv file dump
# Make sure you add .scm when inputting the name for the scheme output file
 


from opencog.atomspace import AtomSpace, TruthValue,Link, Node
import reader
import term

corpus_path = ""

def set_TV(word):
	stv = TruthValue() 
	term_list = term.freq(word,corpus_path)
	if term_list[0] == 0:
		stv.mean = 1/(term_list[1] + 1)
		stv.count = .432  # have no reason for this value
		return stv
	else:
		stv.mean = term_list[0]/term_list[1]
		stv.count = .765 # have no reason for this value 
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
	#global corpus_path
	cn_url= raw_input("Enter ConceptNet csv file address: ")
	corpus_path = raw_input("Enter corpus address: ")
	name_of_scm_file = raw_input("Enter name for the Scheme Output file: ")
	from_file(cn_url,name_of_scm_file)
	print ("Scheme file is created successfully")
	
