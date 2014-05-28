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

- The introduction of the Abduction rule was discussed here:
  - https://github.com/opencog/opencog/pull/777

PLN rules needed:

- GeneralEvaluationToMemberRule
- MemberToInheritanceRule
- DeductionRule
- InheritanceToMemberRule
- MemberToEvaluationRule
- AbductionRule

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
(ConceptNode "Socrates@f250f429-5078-4453-8662-c63cc8f58a22") ; [217]

(ConceptNode "Socrates" (stv .1 1.0)) ; [218]

(ConceptNode "man@80d4e852-1b93-4c49-84e1-5a7352b0dcb1" (stv .1 1.0)) ; [220]

(ConceptNode "men@83d83a2e-940c-46aa-91f1-a7d2c106ef13" (stv .1 1.0)) ; [290]

(ConceptNode "man" (stv .1 1.0)) ; [221]

(ConceptNode "air@e3f175ea-c3d1-45cf-883d-a6d2f6a879ac") ; [292]

(ConceptNode "air") ; [293]

(ConceptNode "present") ; [227]

```

##### Tuple that satisfies breathe(x,y)
```
(ListLink (stv 1.000000 0.000000)
  (ConceptNode "men@83d83a2e-940c-46aa-91f1-a7d2c106ef13") ; [290]
  (ConceptNode "air@e3f175ea-c3d1-45cf-883d-a6d2f6a879ac") ; [292]
) ; [295]
```

##### breathe(x,y)
```
(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "breathe@218b15b2-52a8-430c-93f6-b4bba225418c") ; [287]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "men@83d83a2e-940c-46aa-91f1-a7d2c106ef13") ; [290]
    (ConceptNode "air@e3f175ea-c3d1-45cf-883d-a6d2f6a879ac") ; [292]
  ) ; [295]
) ; [296]
```

##### Socrates IS-A man
```
(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "Socrates@f250f429-5078-4453-8662-c63cc8f58a22") ; [217]
  (ConceptNode "man@80d4e852-1b93-4c49-84e1-5a7352b0dcb1") ; [220]
) ; [223]
```

##### Instance inherits from generic concept
```
(InheritanceLink (stv 1.000000 0.990000)
  (ConceptNode "men@83d83a2e-940c-46aa-91f1-a7d2c106ef13") ; [290]
  (ConceptNode "man") ; [221]
) ; [291]

(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "Socrates@f250f429-5078-4453-8662-c63cc8f58a22") ; [217]
  (ConceptNode "Socrates") ; [218]
) ; [219]

(InheritanceLink (stv 1.000000 0.990000)
  (ConceptNode "man@80d4e852-1b93-4c49-84e1-5a7352b0dcb1") ; [220]
  (ConceptNode "man") ; [221]
) ; [222]
```


#### Inference steps

##### 1) GeneralEvaluationToMemberRule

###### Input
```
(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "breathe@7f5b37e8-e4b3-4335-a06b-68af470cf354") ; [350]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "men@a2905bdd-9214-4717-82c6-dfe21c1263bc") ; [353]
    (ConceptNode "air@9bfe790d-5446-4219-be74-845c0d145fe1") ; [355]
  ) ; [358]
) ; [359]
```

##### Output (and 3 other variations of this link where elements of the ListLink are substituted with variables)
```
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "men@a2905bdd-9214-4717-82c6-dfe21c1263bc") ; [353]
  (SatisfyingSetLink (stv 1.000000 1.000000)
    (VariableNode "$X0") ; [441]
    (EvaluationLink (stv 1.000000 0.000000)
      (PredicateNode "breathe@7f5b37e8-e4b3-4335-a06b-68af470cf354") ; [350]
      (ListLink (stv 1.000000 0.000000)
        (VariableNode "$X0") ; [441]
        (ConceptNode "air@9bfe790d-5446-4219-be74-845c0d145fe1") ; [355]
      ) ; [443]
    ) ; [444]
  ) ; [445]
) ; [446]
```

##### 2) MemberToInheritance Rule

###### Input is previous output
 
###### Output
```
(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "men@a2905bdd-9214-4717-82c6-dfe21c1263bc") ; [353]
  (SatisfyingSetLink (stv 1.000000 1.000000)
    (VariableNode "$X0") ; [441]
    (EvaluationLink (stv 1.000000 0.000000)
      (PredicateNode "breathe@7f5b37e8-e4b3-4335-a06b-68af470cf354") ; [350]
      (ListLink (stv 1.000000 0.000000)
        (VariableNode "$X0") ; [441]
        (ConceptNode "air@9bfe790d-5446-4219-be74-845c0d145fe1") ; [355]
      ) ; [443]
    ) ; [444]
  ) ; [445]
) ; [6107]
```

##### 3) AbductionRule<InheritanceRule>

###### Input
```
(InheritanceLink (stv 1.000000 0.990000)
  (ConceptNode "men@a2905bdd-9214-4717-82c6-dfe21c1263bc") ; [353]
  (ConceptNode "man") ; [284]
) ; [354]
 
(InheritanceLink (stv 1.000000 0.990000)
  (ConceptNode "man@fbc51aff-8074-46d8-b3ba-1ccaeb1adb34") ; [283]
  (ConceptNode "man") ; [284]
) ; [285]
```

###### Output
```
(InheritanceLink (stv 1.000000 0.988901)
  (ConceptNode "man@fbc51aff-8074-46d8-b3ba-1ccaeb1adb34") ; [283]
  (ConceptNode "men@a2905bdd-9214-4717-82c6-dfe21c1263bc") ; [353]
) ; [609]
```

##### 4) DeductionRule<InheritanceRule>

###### Input is previous ouput and
```
(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "Socrates@46ec3d0f-4535-4d01-87b7-84ef65c25a23") ; [280]
  (ConceptNode "man@fbc51aff-8074-46d8-b3ba-1ccaeb1adb34") ; [283]
) ; [286]
```

###### Output
```
(InheritanceLink (stv 1.000000 0.991755)
  (ConceptNode "Socrates@46ec3d0f-4535-4d01-87b7-84ef65c25a23") ; [280]
  (ConceptNode "men@a2905bdd-9214-4717-82c6-dfe21c1263bc") ; [353]
) ; [777]
```

##### 5) DeductionRule<InheritanceRule>

##### Input is previous output and output from 2)

##### Output
```
(InheritanceLink (stv 1.000000 0.986333)
  (ConceptNode "Socrates@46ec3d0f-4535-4d01-87b7-84ef65c25a23") ; [280]
  (SatisfyingSetLink (stv 1.000000 0.000000)
    (VariableNode "$X1") ; [442]
    (EvaluationLink (stv 1.000000 1.000000)
      (PredicateNode "breathe@7f5b37e8-e4b3-4335-a06b-68af470cf354") ; [350]
      (ListLink (stv 1.000000 0.000000)
        (VariableNode "$X1") ; [442]
        (ConceptNode "air@9bfe790d-5446-4219-be74-845c0d145fe1") ; [355]
      ) ; [922]
    ) ; [923]
  ) ; [8605]
) ; [12317]
```

##### 6) InheritanceToMemberRule

###### Input is previous output

###### Output
```
(MemberLink (stv 1.000000 0.989841)
  (ConceptNode "Socrates@46ec3d0f-4535-4d01-87b7-84ef65c25a23") ; [217]
  (SatisfyingSetLink (stv 1.000000 1.000000)
    (VariableNode "$X0") ; [385]
    (EvaluationLink (stv 1.000000 0.000000)
      (PredicateNode "breathe@7f5b37e8-e4b3-4335-a06b-68af470cf354") ; [350]
      (ListLink (stv 1.000000 0.000000)
        (VariableNode "$X1") ; [442]
        (ConceptNode "air@9bfe790d-5446-4219-be74-845c0d145fe1") ; [355]
      ) ; [922]
    ) ; [388]
  ) ; [389]
) ; [6375]
```

##### 5) MemberToEvaluationRule

###### Input is previous output

###### Final output
```
(EvaluationLink (stv 1.000000 0.989841)
  (PredicateNode "breathe@7f5b37e8-e4b3-4335-a06b-68af470cf354") ; [350]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Socrates@46ec3d0f-4535-4d01-87b7-84ef65c25a23") ; [217]
    (ConceptNode "air@9bfe790d-5446-4219-be74-845c0d145fe1") ; [355]
  ) ; [922]
) ; [388]
```

#### Note: Along the way, some other inferences which don't lead to the desired output are produced as well.
