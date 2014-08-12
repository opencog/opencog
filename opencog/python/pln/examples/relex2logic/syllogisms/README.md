# Syllogisms

## Origin

One of the first experiments using PLN has been done on one of the classic
examples of logical reasoning, the categorical syllogism _"Men breathe air.
Socrates is a man. Thus Socrates breathes air."_

By trying to reason with this, some valuable insights have been gained along
the way (which can be viewed [here](../../socrates_demo/README.md)).

Ben thus [suggested](https://groups.google.com/forum/#!topic/opencog/iH0FM2IFEVc)
to attempt to resolve more complex syllogisms in a similar way as they would
arguably lead to more useful conclusions.

The syllogism examples are taken from [here](http://www.fibonicci.com/logical-reasoning/)
and can be viewed [here](http://wiki.opencog.org/w/List_of_Test_Syllogisms).

## Architecture

### PLN reasoning

```test_syllogisms.py``` provides a way to run the syllogisms, similar to the
PLNUnitTester by Alex in ```opencog/tests/python/test_pln/test_rules_new.py```.

The input needs to be in a Scheme file such as ```syllogism-canadian.scm``` where
the input for the chainer is wrapped as follows:
```
(EvaluationLink (PredicateNode "inputs")
    (ListLink
        ; input atoms
    )
)
```
while the rules to be used are specified as follows:
```
(EvaluationLink (PredicateNode "rules")
	(ListLink
		(ConceptNode "ModusPonensRule:ImplicationLink")
		; additional rules
	)
)
```
Finally, the desired output is to be represented as follows:
```
(EvaluationLink (PredicateNode "desired_outputs")
	(ListLink
	    ; desired output atoms
	)
)
```
while atoms that should explicitly not be represented are to be represented as follows:
```
(EvaluationLink (PredicateNode "undesired_outputs")
	(ListLink
	    ; undesired output atoms
	)
)
```

### RelEx2Logic parsing

Whereas ```test_syllogisms.py``` focuses on PLN and requires the input to
specified manually in what would be produced by RelEx2Logic, ```load_r2l```
bridges this gap and takes as input natural language syllogisms (at the moment)
of the form
```
a. 1st premise
b. 2nd premise
|- conclusion
```
and retrieves their RelEx2Logic representation by using the REST API and parsing
these with RelEx and RelEx2Logic in a running CogServer.

Potentially, both modules can be linked to provide a comprehensive
RelEx2Logic-PLN-pipeline which is able to automatically parse and solve a 
plethora of syllogisms and ultimately produce the conclusion in natural language
using NLGen and surface realization.
