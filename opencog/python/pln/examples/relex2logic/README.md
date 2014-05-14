relex2logic / PLN demo
======================

**Incomplete, still in-progress**

Author: Cosmo Harrigan
March 2014

The goal is to demonstrate the simple use of Relex2Logic with PLN to accept
the following sentences as input:

```
Socrates is a man.
Men breathe air.
```

And then use PLN on the logical representation output of Relex2Logic in
order to produce the following output:

```
Socrates breathes air.
```

represented in AtomSpace notation (rather than natural language).

Important notes:

- The demo is functional in that the required output is produced.
  At the moment, this is only done using Python.
  It should be done with RelEx2Logic in the Cogserver.
  To do this, the Relex2Logic rules require modification for "is"/"be"
  predicates, see:
  - https://github.com/opencog/opencog/issues/726

- There are some outstanding issues concerning satisfying sets:
  - https://github.com/opencog/opencog/issues/601
  - https://github.com/opencog/opencog/issues/603
  - https://github.com/opencog/opencog/issues/613

- And some concerning duplicates or bugs in the output:
  - https://github.com/opencog/opencog/issues/733
  - https://github.com/opencog/opencog/issues/734
  - https://github.com/opencog/opencog/issues/735

PLN rules needed:

- EvaluationToMemberRule
- MemberToInheritanceRule
- DeductionRule
- InheritanceToMemberRule
- MemberToEvaluationRule

#### Important note

The representation of "be" should be an InheritanceLink rather than an
EvaluationLink of a PredicateNode if issue #726 (above) is implemented.

The InheritanceLink implementation is assumed for this example.

The example can be loaded into the Cogserver as a MindAgent or run
with Python.

For this example at the moment, concepts are used. These can be
changed for concept instances later.

### The inference process

#### AtomSpace starting contents:

##### Concepts
```
(ConceptNode "Socrates" (av 0 0 0) (stv 0.001000 1.000000)) ; [1]
 
(ConceptNode "man" (av 0 0 0) (stv 0.010000 1.000000)) ; [2]
 
(ConceptNode "air" (av 0 0 0) (stv 0.010000 1.000000)) ; [3]
```

##### Tuple that satifies breathe(x,y)
```
(ListLink (av 0 0 0) (stv 1.000000 0.000000)
  (ConceptNode "man" (av 0 0 0) (stv 0.010000 1.000000)) ; [2]
  (ConceptNode "air" (av 0 0 0) (stv 0.010000 1.000000)) ; [3]
) ; [6]
```

##### breathe(x,y)
```
(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "breathe" (av 0 0 0) (stv 1.000000 0.000000)) ; [5]
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "man" (av 0 0 0) (stv 0.010000 1.000000)) ; [2]
    (ConceptNode "air" (av 0 0 0) (stv 0.010000 1.000000)) ; [3]
  ) ; [6]
) ; [7]
```

##### Socrates IS-A man
```
(InheritanceLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Socrates" (av 0 0 0) (stv 0.001000 1.000000)) ; [1]
  (ConceptNode "man" (av 0 0 0) (stv 0.010000 1.000000)) ; [2]
) ; [4]
```

#### Inference steps

##### 1) EvaluationToMemberRule 

###### Input
```
(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "breathe" (av 0 0 0) (stv 1.000000 0.000000)) ; [5]
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "man" (av 0 0 0) (stv 0.010000 1.000000)) ; [2]
    (ConceptNode "air" (av 0 0 0) (stv 0.010000 1.000000)) ; [3]
  ) ; [6]
) ; [7]
```

##### Output (and 3 other variations of this link where elements of the ListLink are substituted with variables)
```
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "man") ; [2]
  (SatisfyingSetLink (stv 1.000000 1.000000)
    (VariableNode "$X0") ; [144]
    (EvaluationLink (stv 1.000000 0.000000)
      (PredicateNode "breathe") ; [5]
      (ListLink (stv 1.000000 0.000000)
        (VariableNode "$X0") ; [144]
        (ConceptNode "air") ; [3]
      ) ; [146]
    ) ; [147]
  ) ; [148]
) ; [149]
```

##### 2) MemberToInheritance Rule

###### Input is previous output
 
###### Output
```
(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "man") ; [2]
  (SatisfyingSetLink (stv 1.000000 1.000000)
    (VariableNode "$X0") ; [144]
    (EvaluationLink (stv 1.000000 0.000000)
      (PredicateNode "breathe") ; [5]
      (ListLink (stv 1.000000 0.000000)
        (VariableNode "$X0") ; [144]
        (ConceptNode "air") ; [3]
      ) ; [146]
    ) ; [147]
  ) ; [148]
) ; [242]
```

##### 3) DeductionRule<InheritanceRule>

###### Input is previous ouput and
```
[(InheritanceLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Socrates" (av 0 0 0) (stv 0.001000 1.000000)) ; [1]
  (ConceptNode "man" (av 0 0 0) (stv 0.010000 1.000000)) ; [2]
) ; [4]
```

###### Output
```
(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "Socrates") ; [1]
  (SatisfyingSetLink (stv 1.000000 0.000000)
    (VariableNode "$X1") ; [145]
    (EvaluationLink (stv 1.000000 0.000000)
      (PredicateNode "breathe") ; [5]
      (ListLink (stv 1.000000 0.000000)
        (VariableNode "$X1") ; [145]
        (ConceptNode "air") ; [3]
      ) ; [281]
    ) ; [282]
  ) ; [283]
) ; [284]
```

##### 4) InheritanceToMemberRule

###### Input is previous output

###### Output
```
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Socrates") ; [1]
  (SatisfyingSetLink (stv 1.000000 0.000000)
    (VariableNode "$X1") ; [145]
    (EvaluationLink (stv 1.000000 0.000000)
      (PredicateNode "breathe") ; [5]
      (ListLink (stv 1.000000 0.000000)
        (VariableNode "$X1") ; [145]
        (ConceptNode "air") ; [3]
      ) ; [281]
    ) ; [282]
  ) ; [283]
) ; [477]
```

##### 5) MemberToEvaluationRule

###### Input is previous output

###### Final output
```
EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "breathe") ; [5]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Socrates") ; [1]
    (ConceptNode "air") ; [3]
  ) ; [542]
) ; [543]
```

#### Note: Along the way, some other inferences which don't lead to the desired output are produced as well.



### Current output by RelEx & RelEx2Logic (for reference)

### be(Socrates, man)

```
(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "is@a64c0b7f-117e-447c-8879-7601423601b4")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Socrates@7b03b9d8-cdee-4b35-bf29-c6fd35cb4229")
    (ConceptNode "man@35b6f89b-5753-4da1-8dc4-55224cf56789")
  )
)
```

### breathe(men, air)

```
(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "breathe@cd1c5b26-0a62-493f-81eb-e7606d3439fa")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "men@cc05125c-5468-4b86-b2f3-b4cedd2eb29e")
    (ConceptNode "air@4f8ea182-b6a7-4c04-8e1e-c195f5000c8b")
  )
)
```

### Concepts

```
(ConceptNode "man")

(ConceptNode "men@cc05125c-5468-4b86-b2f3-b4cedd2eb29e")

(ConceptNode "air@4f8ea182-b6a7-4c04-8e1e-c195f5000c8b")

(ConceptNode "air")

(ConceptNode "Socrates@7b03b9d8-cdee-4b35-bf29-c6fd35cb4229")

(ConceptNode "Socrates")
```


### Sentences

```
(ReferenceLink (stv 1.000000 1.000000)
  (ParseNode "sentence@6c596aad-8396-43a7-8380-0ab51eb91613_parse_0")
  (ListLink (stv 1.000000 0.000000)
    (WordInstanceNode "Socrates@7b03b9d8-cdee-4b35-bf29-c6fd35cb4229")
    (WordInstanceNode "is@a64c0b7f-117e-447c-8879-7601423601b4")
    (WordInstanceNode "a@13ebe81e-26db-4f40-9bdb-a10c88767830")
    (WordInstanceNode "man@35b6f89b-5753-4da1-8dc4-55224cf56789")
    (WordInstanceNode ".@7b5926db-6913-40a1-bb2c-f265a7435c51")
  )
)

(ReferenceLink (stv 1.000000 1.000000)
  (ParseNode "sentence@061ce111-14e2-4d25-b5e8-f187e02cb093_parse_0")
  (ListLink (stv 1.000000 0.000000)
    (WordInstanceNode "men@cc05125c-5468-4b86-b2f3-b4cedd2eb29e")
    (WordInstanceNode "breathe@cd1c5b26-0a62-493f-81eb-e7606d3439fa")
    (WordInstanceNode "air@4f8ea182-b6a7-4c04-8e1e-c195f5000c8b")
    (WordInstanceNode ".@29183d71-9982-40c4-b474-7e0c2444bfd4")
  )
)
```

### Generalizing from specific instances

```
(InheritanceLink (stv 1.000000 0.000000)
  (PredicateNode "breathe@cd1c5b26-0a62-493f-81eb-e7606d3439fa")
  (PredicateNode "breathe")
)

(InheritanceLink (stv 1.000000 0.000000)
  (ConceptNode "man@35b6f89b-5753-4da1-8dc4-55224cf56789")
  (ConceptNode "man")
)

(InheritanceLink (stv 1.000000 0.000000)
  (ConceptNode "men@cc05125c-5468-4b86-b2f3-b4cedd2eb29e")
  (ConceptNode "man")
)

(InheritanceLink (stv 1.000000 0.000000)
  (ConceptNode "air@4f8ea182-b6a7-4c04-8e1e-c195f5000c8b")
  (ConceptNode "air")
)

(InheritanceLink (stv 1.000000 0.000000)
  (ConceptNode "Socrates@7b03b9d8-cdee-4b35-bf29-c6fd35cb4229")
  (ConceptNode "Socrates")
)

(InheritanceLink (stv 1.000000 0.000000)
  (PredicateNode "is@a64c0b7f-117e-447c-8879-7601423601b4")
  (PredicateNode "be")
)
```
