__author__ = 'Amen Belayneh'

# This code creates a scheme file after inputing the address to a conceptnet csv file dump
# Make sure you add .scm when inputting the name for the scheme output file
 


from opencog.atomspace import AtomSpace, TruthValue,Link, Node
import reader
import mapper

DEFAULT_TV = TruthValue(1,0.5) # 0.5 confidence is used because that is the weight given to most of the assertions on ConceptNet
DEFAULT_TV2 = TruthValue(1,1)

def write_file(cn_assertion, context): # assertion is a list
	if cn_assertion[0] == "/r/IsA":				
		return	'(ContextLink ' + '(stv '+ str(DEFAULT_TV.mean) +' ' +str(DEFAULT_TV.count) +')'  +  ' \n\t' +\
					'(ConceptNode  ' + '"' + str(context) + '"' + ' (stv ' + str(DEFAULT_TV2.mean) +' ' +str(DEFAULT_TV2.count)  +')' + ')\n\t' +\
					'(InheritanceLink ' + '(stv '+ str(DEFAULT_TV.mean) +' ' +str(DEFAULT_TV.count) +')'  + '\n\t\t' +\
						'(ConceptNode  ' + '"' + cn_assertion[1][6:] + '"' + ' (stv '+ str(DEFAULT_TV2.mean) +' ' +str(DEFAULT_TV2.count)  +')' + ')\n\t\t' +\
						'(ConceptNode  ' + '"' + cn_assertion[2][6:] + '"' + ' (stv '+ str(DEFAULT_TV2.mean) +' ' +str(DEFAULT_TV2.count)  +')' + ')\n\t' +\
					')\n'	+\
				')'
		
	else:
		return	'(ContextLink ' + '(stv '+ str(DEFAULT_TV.mean) +' ' +str(DEFAULT_TV.count)  +')' +  ' \n\t'  +\
					'(ConceptNode  ' + '"' + str(context)+ '"' + ' '  + '(stv '+ str(DEFAULT_TV2.mean) +' ' +str(DEFAULT_TV2.count)  +')' + ')\n\t' +\
					'(EvaluationLink  ' + '(stv '+ str(DEFAULT_TV.mean) +' ' +str(DEFAULT_TV.count) +')'  + '\n\t\t' +\
						'(PredicateNode  ' + '"' + cn_assertion[0][3:] + '"' +' ' + ' (stv '+ str(DEFAULT_TV.mean) +' ' +str(DEFAULT_TV.count)+ ')' + ')\n\t\t' +\
						'(ListLink  '	+ '\n\t\t\t' +\
								'(ConceptNode  ' + '"' + cn_assertion[1][6:] + '"' + ' (stv '+ str(DEFAULT_TV2.mean) +' ' +str(DEFAULT_TV2.count)  +')' + ')\n\t\t\t' +\
								'(ConceptNode  ' + '"' + cn_assertion[2][6:] + '"' +' (stv '+ str(DEFAULT_TV2.mean) +' ' +str(DEFAULT_TV2.count)  +')' + ')\n\t\t' +\
						')\n\t'	 +\
					')\n' +\
				')'


def from_file(file_path, file_name):
	lists_of_assertions = reader.csv(file_path) # lists_of_assertions is a list of list of assertion
	with open(file_name,'w') as scm_file:
		for an_assertion in lists_of_assertions:
			if an_assertion[3] == "/ctx/all":
				temp = write_file(an_assertion, "Universe")
				scm_file.write(temp+'\n'*2)
			
		else: # for other type of context naming is same as used in ConceptNet
			#map(an_assetion,an_assertion[3][5:],atomspace)
			pass	
	
			
if __name__ == '__main__':
	url= raw_input("Enter file address: ")
	name_of_scm_file = raw_input("Enter name for the Scheme Output file: ")
	from_file(url,name_of_scm_file)
	print ("Scheme file is created successfully")
	
