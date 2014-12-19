Unified rule engine
-------------------

**Introduction**

   The unified rule engine project aims at building a generic opencog rule engine on top of the pattern matcher with a c++ PLN
 implementation where rules are put in a scheme representation. This will enable the reuse PLN for any kind of Backward
 and Forward chaining inferences as long as rules are represented as an ImplicationLinks or BindLinks which are decoupled  from 
 the PLN inference engine and are loaded dynamically from a scheme file.In the new design of the rule engine, PLN will use the Pattern matcher API
 for querying the atomspace which will also guarantees code reuse and no reinvention of the wheel.The pattern matcher can be invoked implicitly with the default
 callbacks or explicitly using custom callback handlers for specialized cases.All the above criteria and other issues has required a new implementation of PLN. 

**Overall requirement/objectives**
  
  1.Rules should be specified as ImplicationLinks or BindLinks in a scheme file.  
  2.Mechanisms will be provided for backward chaining from a given target, or forward chaining from a set of premises.  
  3.The control mechanism, i.e. the policy for choosing which rules to apply in a given case,
   must be pluggable. That is, when you invoke the rule engine, you get to choose which of the
   available control policies will be used.   
  4.There should be a standard way to express exclusivity between rules,and priority levels of rules --- but both of
   these things should be relative to a given control policy (i.e rule X might have higher priority than rule Y with 
   respect to policy A, but not policy B)   
  5.The rule engine should be associated with some way of keeping track of which rules have been applied to which Atoms.
    This information may be used by some control policies.   
  6.Use pattern matcher for finding groundings and implement the callbacks if the need arises.
    
  further reading  
  [http://wiki.opencog.org/w/Unified_Rule_Engin](http://wiki.opencog.org/w/Unified_Rule_Engin)  
  [http://wiki.opencog.org/w/Control_policy](http://wiki.opencog.org/w/Control_policy)   
  [http://wiki.opencog.org/w/Pattern_Matcher](http://wiki.opencog.org/w/Pattern_Matcher)
   
 **New PLN implementation overview**  
  In general PLN rules and their associated formulas are all supposed to be ported to a scheme representation and are loaded at 
  the beginning of the inference process(backward and forward chaining). most of PLN formulas have been ported
in to a scheme file by contributors and can be found [here](https://github.com/opencog/opencog/tree/master/opencog/reasoning/RuleEngine/rules).

  The high level algorithm for the new PLN forward and backward chaining is found [here](http://wiki.opencog.org/w/New_PLN_Chainer_Design).
  
**Algorithmic detail of the current implementation (Dec 2014)**
  
***Forward chaining***

The implementation of the forward chainer is pretty straight forward.the call graph starts in
the do_chain method.I have tried to use config files as a ways to declare what rules to use during the chaining process. the rules are read
by load_fc_conf method. The current state of the code does the forward chaining algorithm stated in the wiki page with the exceptions of 
 
Below is a kind of pseudo code of the do_bc function

	do_chain(htarget)
		hcurrent_target	
		steps = 0
		while (steps <= ITERATION_SIZE /*or !terminate*/) 
			if steps == 0
				if htarget == Handle::UNDEFINED
					hcurrent_target = choose_target_from_atomspace(main_atom_space); //start FC on a random target
				else
					hcurrent_target = htarget;
	    	else 
				if (!target_list_.empty())
					hcurrent_target = choose_target_from_list(target_list_)
			choose_input(hcurrent_target); //add more premise via pattern matching of related atoms to hcurrent_target
			choose_rule()
			patternmatch(hcurrent_chosen_rule) // call the pattern matching with the chosen rule wrapped in a BindLink
			steps++
	
 *Implementing fitness based rule choosing:For the sake of having a prototype implementation other parts of the algorithms, I have implemented the choose_rule as a random selector. 
 
 *The termination criteria: right now it just does some constant iterations specified in the config file which should be changed to appropriate criteria. 
 
 *The control policy: I did a basic rule loading from a config file as a prototype. But I am thinking of how to declare conditional rule applications. An example usage scenario is
 
 the relex to logic SV and SVO rules. the detail is found [here](http://wiki.opencog.org/w/RelEx2Logic_Rules#Suggested_Rule_File_Format).

how to call the forward chainer from a scheme interface?
One can use the cog-fc scheme binding to start forward chaining on a particular target.
Example: suppose there is some knowledges about the ConceptNode Socrates then one can do a bunch of forward chaining inference
by calling (cog-fc (ConceptNode "Socrates")) from a scheme shell or interfaces. All results of the inferences are returned in 
in a ListLink.

***Backward chaining***

In the backward chaining inference we are interested in either truth value fulfillment query or variable fulfillment query.The current implementation does
the later where a variable containing link is passed as an argument and the backward chainer tries to find grounding for the variable.The entry point 
for the backward chainer is the do_bc function. A pseudo code representation would look like
 
 		vector<map<Handle,HandleSeq>> inference_list
 		
 		function map<var,soln> do_bc(target) 
    		HandleSeq kb_result = query_knowledge_base(target) //try to find grounding via pattern matching    
    	if kb_result is empty    
    		HandleSeq rules = query_rule_base(target)    	
    		if rules is not empty
    			rule = rules[0]
    			inference_list.push(unify(target,implicand(rule))) //map variables in the target to matching nodes in the rule( which might be variables)
    			map<var,soln> solution = backward_chain(implicand,rule)
				inference_list.push(solution)
    		else
    			return empty_soln
    	
    	function map<var,soln> backward_chain(target,rule)
    		root_logical_link = get_root_logical_link(rule) //see if the rule has logical connectors (right now AND and OR)
    		if root_logical_link = null
    			implicand = outgoing_set(rule)[0] 
    			return do_bc(implicand)
    		//build a tree of the the logical links and premises( premises could by themselves be a logical link) as a map	
    		map<Handle, HandleSeq> logical_link_premise_map = get_logical_link_premises_map(rule)
			
			map<Handle, map<Handle, HandleSeq>> premise_var_ground_map
			HandleSeq visited_logical_link
			HandleSeq evaluated_premises
			visited_logical_link.push_back(root_logical_link); //start from the root

			//go down deep until a logical link maps only to set of premises in the logical_link_premise_map object
			//e.g start from (and x y) instead of (and (and x y) x) and start backward chaining  from there. i.e bottom up.
			while visited_logical_link is not empty
				logical_link = visited_logical_link.back();
				visited_logical_link.pop_back();

				HandleSeq premises = logical_link_premise_map[logical_link];
				Handle llink = get_unvisited_logical_link(premises, evaluated_premises);
				if llink != null
					visited_logical_link.push_back(llink)
					continue
				for (Handle premise : premises) 
					p = find premise in evaluated_premise
					if p == null 
						var_grounding = do_bc(premise);
						premise_var_ground_map[premise] = var_grounding
						evaluated_premises.push_back(premise)	
		
				 = join_premise_vgrounding_maps(logical_link,premise_var_ground_map)
				results.push_back(v) //add to results
				evaluated_premises.push_back(logical_link)
			return ground_target_vars(target, results);  //unify vars in target to grounding in results(basically chase the variable pointing variables till there is a grounding or no more map)    		    		
    	 
  The backward chainer can be used by calling the cog-bc scheme binding.Example
  
  		(define whose-frog (InheritanceLink (VariableNode "$whosFrog")(ConceptNode "Frog"))) 
  		(cog-bc whose-frog)  
  will return all the solution for the variable $whosFrog in a ListLink of ListLinks. I did this thinking of multivariable grounding with multiple solutions for each variable. 			  	

**Control policy**  
The control policy is implemented as [config](https://github.com/misgeatgit/opencog/blob/master/lib/rule-engine.conf) file containing what rules and other parameters to load. like max iteration being defined in .
the configuration is being read in the load_fc_conf() method of the ForwardChainer.cc.Admittedly  this is the least properly done as an initial prototype.I want to make it more robust and decoupled.I am thinking on it.     

**Summary of current state of the implementation**

The rule engine as it exists now is in its infancy. So far I have been able to write a forward chainer,backward chainer and a configuration system ( basically it loads rules from a configuration file)
There is a lot of space for improvement.Right now am working on

*Rethinking the design configuration/control policy so that it complies with the initial design goal

*Adding capability of multiple inference tree in the backward chainer

*Storing inference history

*Truth value fulfillment queries

*Rule choosing fitness functions

*Inference termination 

**Refactoring out some codes  





***Author*** *Misgana Bayetta*
