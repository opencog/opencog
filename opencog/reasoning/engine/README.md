Unified rule engine
-------------------

## Introduction

The unified rule engine project aims at building a generic opencog
rule engine on top of the Pattern Matcher with a C++ PLN
implementation where rules are put in a scheme representation. This
will enable the reuse of PLN for any kind of Backward and Forward
chaining inferences, as long as rules are represented as an
ImplicationLinks or BindLinks which are decoupled from the PLN
inference engine and are loaded dynamically from a scheme file.  In
the new design of the rule engine, PLN uses the Pattern Matcher API
for querying the atomspace to guarantee code reuse and no reinvention
of the wheel.  The pattern matcher can be invoked implicitly with the
default callbacks or explicitly using custom callback handlers for
specialized cases.  All the above criteria and other issues has
required a new implementation of PLN.

## Overall requirement/objectives

1. Rules should be specified as ImplicationLinks or BindLinks in a
   scheme file.

2. Mechanisms will be provided for backward chaining from a given
   target, or forward chaining from a set of premises.

3. The control mechanism, i.e. the policy for choosing which rules to
   apply in a given case, must be pluggable. That is, when you invoke
   the rule engine, you get to choose which of the available control
   policies will be used.

4. There should be a standard way to express exclusivity between
   rules,and priority levels of rules --- but both of these things
   should be relative to a given control policy (i.e rule X might have
   higher priority than rule Y with respect to policy A, but not
   policy B)

5. The rule engine should be associated with some way of keeping
  track of which rules have been applied to which Atoms.  This
  information may be used by some control policies.

6. Use pattern matcher for finding groundings and implement the
   callbacks if the need arises.

Further reading:

  [http://wiki.opencog.org/w/Unified_Rule_Engine](http://wiki.opencog.org/w/Unified_Rule_Engine)

  [http://wiki.opencog.org/w/Control_policy](http://wiki.opencog.org/w/Control_policy)

  [http://wiki.opencog.org/w/Pattern_Matcher](http://wiki.opencog.org/w/Pattern_Matcher)

## New PLN implementation overview

In general, PLN rules and their associated formulas are all supposed
to be ported to a scheme representation and are loaded at the
beginning of the inference process (backward and forward
chaining). Most of PLN formulas have been ported in to a scheme file
by contributors and can be found
[here](https://github.com/opencog/opencog/tree/master/opencog/reasoning/engine/rules).

The high level algorithm for the new PLN forward and backward chaining
is found [here](http://wiki.opencog.org/w/New_PLN_Chainer_Design).
  
## Algorithmic detail of the current implementation (Dec 2014)

### Forward chaining

The implementation of the forward chainer is pretty straight forward.
The call graph starts in the do_chain method.  I have tried to use
config files as a ways to declare what rules to use during the
chaining process. The rules are read by `load_fc_conf` method. The
current state of the code does the forward chaining algorithm stated
in the wiki page.
 
Below is a kind of pseudo code of the do_chain function

	do_chain(hsource)
		hcurrent_source	
		steps = 0
		while (steps <= ITERATION_SIZE /*or !terminate*/) 
			if steps == 0
				if hsource == Handle::UNDEFINED
					hcurrent_source = choose_source_from_atomspace(main_atom_space); //start FC on a random source
				else
					hcurrent_source = hsource;
	    	else 
				if (!source_list_.empty())
					hcurrent_source = choose_source_from_list(source_list_)
			choose_input(hcurrent_source); //add more premise via pattern matching of related atoms to hcurrent_source
			choose_rule()
			patternmatch(hcurrent_chosen_rule) // call the pattern matching with the chosen rule wrapped in a BindLink
			steps++
	
* Implementing fitness based rule choosing: For the sake of having a
  prototype implementation other parts of the algorithms, I have
  implemented the choose_rule as a random selector.
 
* The termination criteria: right now it just does some constant
  iterations specified in the config file which should be changed to
  appropriate criteria.
 
* The control policy: I did a basic rule loading from a config file as
  a prototype. But I am thinking of how to declare conditional rule
  applications. An example usage scenario is
 
The relex to logic SV and SVO rules. the detail is found
[here](http://wiki.opencog.org/w/RelEx2Logic_Rules#Suggested_Rule_File_Format).

#### How to call the forward chainer from a scheme interface?

One can use the `cog-fc` scheme binding to start forward chaining on a
particular source.

**Example**: suppose there is some knowledges about the ConceptNode
Socrates then one can do a bunch of forward chaining inference by
calling `(cog-fc (ConceptNode "Socrates"))` from a scheme shell or
interfaces. All results of the inferences are returned in in a
ListLink.

### Backward chaining

In the backward chaining inference we are interested in either truth
value fulfillment query or variable fulfillment query.  The current
implementation does the later where a variable containing link is
passed as an argument and the backward chainer tries to find grounding
for the variable.  The entry point for the backward chainer is the
`do_full_chain` or the `do_step` function.

Here's how the criminal example located at
https://github.com/opencog/opencog/blob/master/opencog/python/pln_old/examples/backward_chaining/criminal.scm
is expected to be solved by the Backward Chainer, when only the Modus
Ponens rule is present.


```
t: InhLink $who criminal
-> kb matched

t: InhLink $x crimainl, InhLink $who criminal
-> kb match fail
-> match modus ponens rule
-> output matched: VarNode $B-1 => InhLink $x criminal
-> input became: (AndLink (ImpLink (VarNode $A-1) (QuoteLink (InhLink $x criminal))) (VarNode $A-1))
-> premises selection
-> none of the premises can be grounded to solve for $x
-> add to targets

t: (ImpLink (AndLink ... american ... weapon ...) (InhLink $x criminal)), (AndLink ... american ... weapon ...), InhLink $x crimnal
-> no free var

t: (AndLink ... american ...), InhLink $x criminal, InhLink $who criminal
-> kb match fail
-> break apart the AndLink

t: InhLink $x American, InhLink $y weapon, EvaLink sell $x $y $z, InhLink $z hostile, InhLink $x criminal, InhLink $who criminal
-> kb matched

t: InhLink $y weapon, EvaLink sell $x $y $z, InhLink $z hostile, InhLink $x criminal, InhLink $who criminal
-> kb matched

t: EvaLink sell $x $y $z, InhLink $z hostile, InhLink $x criminal, InhLink $who criminal
-> kb match sell West $a Nono
-> got free var, add to target

t: EvaLink sell West $a Nono, InhLink $z hostile, InhLink $x criminal, InhLink $who criminal
-> kb match fail
-> match modus ponens rule
-> output matched: VarNode $B-2 => EvaLink sell West $a Nono
-> input became: (AndLink (ImplicationLink (VarNode $A-2) (QuoteLink (EvaLink sell West $a Nono))) (VarNode $A-2))
-> premises selection
-> one of the premises can be grounded by missile@123
-> forward chain added (EvaLink sell West missle@123 Nono) to atomspace
-> no premises with free var, this target is solved

t: InhLink $z hostile, InhLink $x criminal, InhLink $who criminal
-> kb match

t: InhLink $b hostile, InhList $z hostile, InhLink $x criminal, InhLink $who criminal
-> kb match fail
-> match modus ponens rule
-> output matched: VarNode $B-3 => InhLink $b hostile
-> input became: (AndLink (ImplicationLink (VarNode $A-3) (QuoteLink (InhLink $b hostile))) (VarNode $A-3))
-> premises selection
-> one of the premises can be grounded by Nono
-> forward chain added (InhLink Nono hostile) to atomspace
-> no premises with free var, this target is solved

t: InhList $z hostile, InhLink $x criminal, InhLink $who criminal
-> kb match

t: InhLink $x criminal, InhLink $who criminal
-> kb match fail
-> matched modus poenes rule
-> output matched: VarNode $B-4 => InhLink $x criminal
-> input became: (AndLink (ImpLink (VarNode $A-4) (QuoteLink (InhLink $x criminal))) (VarNode $A-4))
-> premises selection
-> one of the premises can be grounded by West
-> forward chain added (InhLink West criminal) to atomspace
-> no premises with free var, this target is solved

t: InhLink $who criminal
-> kb match

$who in the end map to West

```

where `t` is the targets stack (left is the front).  Since there's a
check where a target will not be readded to the stack if all its
knowledge base matches are in the inference history or has no free
var, persumably the targets stack will be cleared and get back to the
original target `(InheritanceLink $who criminal)` and finally solve
`$who`

## Control policy

The control policy is implemented as
[json](https://github.com/opencog/opencog/blob/master/opencog/reasoning/engine/default_cpolicy.json)
file containing what rules and other parameters to load (like the
maximum number of iterations).  the configuration is being read in the
`load_fc_conf()` method of the ForwardChainer.cc.  Admittedly this is
the least properly done as an initial prototype.  I want to make it
more robust and decoupled.  I am thinking about it.

## Summary of current state of the implementation

The rule engine as it exists now is in its infancy. So far I have been
able to write a forward chainer, backward chainer and a configuration
system ( basically it loads rules from a configuration file). There is
a lot of space for improvement. Right now am working on

* Rethinking the design configuration/control policy so that it
  complies with the initial design goal

* Adding capability of multiple inference tree in the backward chainer

* Storing inference history

* Truth value fulfillment queries

* Rule choosing fitness functions

* Inference termination 

* Refactoring out some codes

## Rule represenation next steps

- See if these can be implemented to directly use the "side-effect
  free" versions so that the truth value application occurs inside the
  ImplicationLink rather than inside the Scheme rule. This was
  discussed [here](https://groups.google.com/d/msg/opencog/KUptHRvBXu0/YR6oySxLKeMJ).

- See if the link type can be made to allow a dynamic list of valid
  link types. For example, for the Deduction Rule: {InheritanceLink,
  SubsetLink, ImplicationLink, ExtensionalImplicationLink}

- Support all the TruthValue types

- Utilize a graph rewriting unit test framework, that is currently
  being discussed, to assert that the replacement graphs match a
  predefined expected value for specific test instances

- For a rule like Modus Ponens to work, it will be necessary to
  implement "Recursive Unification using the Pattern Matcher",
  described [in this thread](http://wiki.opencog.org/w/Idea:_Recursive_Unification_using_the_Pattern_Matcher).

- Rules will require mutual exclusions and priorities, but that can
  likely be implemented outside of the definition of the rule itself
  for clarity.

***Author*** *Misgana Bayetta*
