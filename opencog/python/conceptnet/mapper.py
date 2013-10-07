__author__ = 'Amen Belayneh'

# This code maps the conceptnet relations to OpenCog atoms

from opencog.atomspace import AtomSpace, TruthValue
import reader

DEFAULT_TV = TruthValue(1,0.5) # 0.5 confidence is used because that is the weight given to most of the assertions on ConceptNet
DEFAULT_TV2 = TruthValue(1,1)

def map(cn_assertion, context,atomspace): # cn_assertion is a list
	if cn_assertion[0] == "/r/IsA":
				
		atomspace.add_link('ContextLink',
			[atomspace.add_node('ConceptNode',context, DEFAULT_TV2),
			atomspace.add_link('InheritanceLink',
				[atomspace.add_node('ConceptNode', cn_assertion[1][6:], DEFAULT_TV2), atomspace.add_node('ConceptNode', cn_assertion[2][6:], DEFAULT_TV2)
				], DEFAULT_TV)
			],DEFAULT_TV)
	else:
		atomspace.add_link('ContextLink',
			[atomspace.add_node('ConceptNode',context, DEFAULT_TV2),
			atomspace.add_link('EvaluationLink',
				[atomspace.add_node('PredicateNode',cn_assertion[0][3:],DEFAULT_TV),
				atomspace.add_link('ListLink',
					[atomspace.add_node('ConceptNode', cn_assertion[1][6:], DEFAULT_TV2), atomspace.add_node('ConceptNode', cn_assertion[2][6:], DEFAULT_TV2)
					])
				])
			],DEFAULT_TV)
		
def map_from_path(file_path,atomspace):
	lists_of_assertions = reader.csv(file_path) # lists_of_assertions is a list of lists
	for an_assertion in lists_of_assertions:
		if an_assertion[3] == "/ctx/all":
			map(an_assertion, "Universe", atomspace)
			
		else: # for other type of context naming is same as used in ConceptNet
			#map(an_assetion,an_assertion[3][5:],atomspace)
			pass

			
if __name__ == '__main__':
	url= raw_input("Enter file address: ")
	atomspace = AtomSpace()
	map_from_path(url,atomspace)
	
