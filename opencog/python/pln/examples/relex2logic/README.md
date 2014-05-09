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

- We need to modify PLN to handle satisfying sets properly, see:
  - https://github.com/opencog/opencog/issues/601
  - https://github.com/opencog/opencog/issues/602
  - https://github.com/opencog/opencog/issues/603
  - https://github.com/opencog/opencog/issues/613
  - https://github.com/opencog/opencog/pull/637

- The Relex2Logic rules require modification for "is"/"be" predicates, see:
  - https://github.com/opencog/opencog/issues/726

PLN rules needed:

- GeneralEvaluationToMemberRule
- MemberToInheritanceRule
- DeductionRule

## Logical output from Relex2Logic:

### Predicates

#### Important note

The representation of "be" will change to an InheritanceLink rather than an EvaluationLink of a PredicateNode if issue #2 (above) is implemented.

That representation would appear as:

```
(InheritanceLink
    (ConceptNode "Socrates@7b03b9d8-cdee-4b35-bf29-c6fd35cb4229")
    (ConceptNode "man@35b6f89b-5753-4da1-8dc4-55224cf56789"))
```

However, the current representation produced is as follows.

#### Current representation

```
(PredicateNode "breathe")

(PredicateNode "be")
```

##### be(Socrates, man)

```
(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "is@a64c0b7f-117e-447c-8879-7601423601b4")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Socrates@7b03b9d8-cdee-4b35-bf29-c6fd35cb4229")
    (ConceptNode "man@35b6f89b-5753-4da1-8dc4-55224cf56789")
  )
)
```

##### breathe(men, air)

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
