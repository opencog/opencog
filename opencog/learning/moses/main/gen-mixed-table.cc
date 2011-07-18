#include <iostream>

#include <boost/lexical_cast.hpp>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/random.h>

#include <opencog/comboreduct/combo/eval.h>
#include <opencog/comboreduct/combo/table.h>
#include <opencog/comboreduct/combo/type_tree.h>
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>

using namespace std;
using namespace boost;
using namespace opencog;
using namespace combo;
using namespace ant_combo;


int main(int argc, char ** argv)
{
    int rand_seed, nsamples_contin,nsamples_bool;
    float partial_probability = 1.0; 
    double  max_randvalue, min_randvalue;
    combo_tree combo_tr;
    type_tree tr;
    argument_type_list arg_type_list;
    try {
        if (argc != 5 && argc != 6)
            throw "foo";
        rand_seed = lexical_cast<int>(argv[1]);
        max_randvalue = lexical_cast<double>(argv[2]);
        min_randvalue = lexical_cast<double>(argv[3]);
        nsamples_contin = lexical_cast<int>(argv[4]);
        if(argc == 6)
        	partial_probability=lexical_cast<float>(argv[5]);
    } catch (...) {
        
        cout << "Description:" << endl;
        cout << "\t>=0 && partial_probability <=1gen-mixed-table can generate the complete or partial table of the boolen variables with"
             << "a certain number of samples for the set of contionuous int variables for each instance in the boolean table"<<endl;
        cout << "\tThis results in a sample set of size boolean_table_size X n_continous_samples_per_instance"<<endl;               
        cout << "\tA special case is when the probability (the last input parameter) is set to -1. In this case, the number of"<<endl;
        cout << "\t samples generated is equal to the n_contin_samples. For each contin sample for the conting varaibles "<<endl;
        cout << "\twe take a random boolean sample for the boolean variables."<<endl;  
        cout << "Usage:" << endl;
        cout << "\t\t" << "1. complete table: " << argv[0] << " rand_seed max_randvalue min_randvalue n_continous_samples_per_instance " << endl;
        cout << "\t\t" << "2. partial  table: " << argv[0] << " rand_seed max_randvalue min_randvalue n_continous_samples_per_instance partial_probability" << endl;
        cout << "\t\t" << "3. contin_samples  table: " << argv[0] << " rand_seed max_randvalue min_randvalue n_continous_samples_per_instance -1" << endl;
        exit(1);
    }

    while (cin.good()) {
        cout << "Please input the combo program :" << endl;
        // get the combo_tree from the stream
 
        cin >> combo_tr;
 
        if (!cin.good())
            break;
 
        // infer the tree type and get the argument_type_list
        tr = infer_type_tree(combo_tr);
        if(!is_well_formed(tr))
        {
        	cout<<"type tree is incorrect"<<endl;
        	continue;
        }
        
        size_t contin_arg_number = contin_arity(tr);
        size_t boolean_arg_number = boolean_arity(tr);
        
        arg_type_list = type_tree_input_arg_types(tr);
        size_t arg_number = arg_type_list.size();
        int arg_number_bls = 1 << arg_number; //2^arg_number
		
        // generate the RandNumber for the given the max_radvalue and min_randvalue
        MT19937RandGen rng(rand_seed);
        contin_input_table rands(nsamples_contin, contin_arg_number, rng, max_randvalue, min_randvalue);

		try{
			if(contin_arg_number==0){ //if the function is purely boolean
				if(partial_probability >=0 && partial_probability <=1){
            		//  generate the truth-table
	            	nsamples_bool = arg_number_bls;
    	        	complete_truth_table truthtable(combo_tr, nsamples_bool);
	
    	        	// output the truth-table
        		   	for ( int i = 0 ; i < nsamples_bool; ++i) {
	               	 bool is_output = biased_randbool(partial_probability, rng);

    	            	if (is_output) {
        	        		    for ( size_t j = 0; j < arg_number; ++j) {
            	     	       cout << ( (i >> j) % 2 ) << "\t";
                	 	   }
                 		   std::cout << static_cast<int>(truthtable[i]) << endl;
                		}    	         
        	    	}
				}
        	    else if(partial_probability == -1 )
        	    {
        	    	// we stick to the usage 3. (the total number of samples is the one defined as parameter, nr_samples_conting)
        	    	if(nsamples_contin > arg_number_bls)
        	    		nsamples_bool = arg_number_bls;
       	    		else
       	    			nsamples_bool = nsamples_contin;
       	    			complete_truth_table truthtable(combo_tr, nsamples_bool);
        	    		        	    
        	    	for ( int i = 0 ; i < nsamples_bool; ++i) {        	    		              	
						for ( size_t j = 0; j < arg_number; ++j) 
            	     		cout << ( (i >> j) % 2 ) << "\t";
                		std::cout << static_cast<int>(truthtable[i]) << endl;
                	}    	         
        	    	
        	    }
        	    else
        	    	cout <<"wrong value in the last parameter. The partial_probability value should either be -1 or from [0,1]"<<endl;        	    	 
        	}         	
        	else{
    	   	
    	   		//this is to generate also the variables which are not present in the method. I make them of type contin
                contin_arg_number=arg_number-boolean_arg_number;
		        nsamples_bool = arg_number_bls; //@todo: maybe there is a bug here, it may be pow(2, boolean_arg_number)
        		//generate the mixed table        	
        		mixed_table mt(combo_tr, rands,tr, rng);
        	
	        	//fetch the evaluation of the output for each sample
    	    	std::vector<variant<bool, contin_t> > vt=mt.get_vt();
        		std::vector<variant<bool, contin_t> >::const_iterator il = vt.begin();

	        	//get the position of each variable in the list of arguments	
    	    	boost::unordered_map<int, int> arg_idx_map=mt.get_arg_idx_map();
        		//variable to keep track of each boolean variable
        		int bool_arg_count;
    	    	if(partial_probability >=0 && partial_probability<=1){        	
        			for(int j = 0 ; j < nsamples_bool; ++j) {
        				bool is_output =
                            biased_randbool(partial_probability, rng);
        				for(const_cm_it i = rands.begin(); i != rands.end(); ++i) {
        				
	        				bool_arg_count=j;        			       			
    	    				const_cv_it contin_it = (*i).begin();    
        					argument_type_list_it it = arg_type_list.begin() ;
        				    if(is_output){    								
             					for( unsigned int k =0; k<arg_number;++k) { 			  			
             						type_tree_pre_it tp_it = (*(it+k)).begin();          		
             						if((*tp_it)== id::boolean_type){             				
             							cout<<bool_arg_count%2<<"  ";             					
             							bool_arg_count/=2;             				
             						}	
             						else{
             							int idx=arg_idx_map[k]; // find the right position of the variable in the rands table            				
	             						std::cout<<*(contin_it+idx)<<"  ";             				
    	         					}             			
        	     				}
      							std::cout<< (*il)<<endl;
        			    	}
            				if (il!=vt.end())
	            				++il;
    	    			}
        			}	        	
        		}			
				else if (partial_probability == -1)
				{
					int contin_indx=0,bool_indx;
					for (const_cm_it i = rands.begin(); i != rands.end(); ++i, ++contin_indx) {
        			
	    	   			bool_indx=rng.randint(nsamples_bool); 
	    	   			bool_arg_count=bool_indx;       			       			
       					const_cv_it contin_it = (*i).begin();    
       					argument_type_list_it it = arg_type_list.begin() ;
       			       								
      					for( unsigned int k =0; k<arg_number;++k) { 			  			
       						type_tree_pre_it tp_it = (*(it+k)).begin();          		
       						if((*tp_it)== id::boolean_type){             				
       							cout<<bool_arg_count%2<<"  ";             					
       							bool_arg_count/=2;             				
       						}		
       						else{
       							int idx=arg_idx_map[k]; // find the right position of the variable in the rands table            				
           						std::cout<<*(contin_it+idx)<<"  ";             				
         					}             			
   	     				}
   	     				//find the right postion of the output in the mixed table that correspondent to this sample of input values and print it
   	     				
						std::cout<< *(il+bool_indx*nsamples_contin+contin_indx)<<endl; 
   			    	}   			    	    			
				}
				else
        		    cout <<"wrong value in the last parameter. The partial_probability value should either be -1 or from [0,1]"<<endl;	
        	}     
        } catch (...) {
            cout << "an exception has been raised perhaps you should try again with different "
                 "parameters" << endl;
            exit(1);
        }        
    }    
    return 0;
}

