Pattern Miner
=============

New pattern miner implementation which should ultimately replace
Shujing Ke pattern miner (see `learning/README.md`). In its current
state is it very comprehensive, that is it shouldn't miss any desired
pattern. However, due to that, it can also be very slow, possibly
slower than Shujing Key pattern miner depending on the usage.
Additional heuristics and filtering techniques will be implemented in
as time goes.

If you know what you are dealing with and want to use it, jump
straight to the Usage Section.

Problem and Terminology
-----------------------

The pattern miner attempts to solve the problem of finding frequent
patterns in the AtomSpace. The terminology used here is similar to the
one defined in this overview [1] and the algorithm mimics the typical
algorithms of the subtree mining litterature with the additional twist
that patterns are Atomese programs.

Let us recall the important terms

* Text tree: a tree that is part of the data set to be mined.
* Pattern tree: a tree representing a pattern, that is capturing a
  collection of text trees.
* Frequency (of a pattern): number of text trees and subtrees matching
  a given pattern.
* Support (of a pattern): similar to frequency.
* Minimum support: parameter of the mining algorithm to discard
  patterns with frequency below that value.
* A priori property: assumption that allows to systematically prune
  the search space. In its least abstract form, it expresses the fact
  that if a pattern tree has a certain frequency `f` then a
  specialization of it can only have a frequency that is equal to or
  lower than `f`.

Algorithm
---------

Patterm mining operates by searching the space of pattern trees,
typically starting from the most abstract pattern, the one that
encompass all text trees, construct specializations of it, retain
those that have enough support (frequency equal to or above the
minimum support), then recursively specialize those, and so on.

### Pattern Trees in Atomese

Here pattern trees are Atomese programs. So for instance the most
abstract pattern, called Top, is the following program
```
(Lambda (Variable "$X") (Variable "$X"))
```
that is the identity. When passed to the pattern matcher, it results
in program that matches all atoms in the AtomSpace.

As another example, a pattern matching only `Inheritance` links would
look like
```
(Lambda
  (VariableList
    (Variable "$X")
    (Variable "$Y"))
  (Inheritance
    (Variable "$X")
    (Variable "$Y")))
```

Or, slightly more specialized, a pattern matching only `Inheritance`
links with the source and target would look like
```
(Lambda
  (Variable "$X")
  (Inheritance
    (Variable "$X")
    (Variable "$Y")))
```

### Algorithm Sketch

Given a collection of text trees `T`, a minum support `ms` and an
initial collection of patterns `C` (containing at least the identity
pattern, Top), the pattern mining algorithm works as follows

1. Select a pattern `P` from `C`
2. Extract the valuation set of `P` over `T`, called `V`
3. Determine the shallow abstractions of `V`, called `A`
4. Specialize `P` by composing it with elements in `A`
5. Add the resulting specializations with enough support in `C`,
   discard the others
6. Repeat till termination

Let us now detail each step

#### Step 2: Extract Valuation Set

The valuation set of a pattern `P` over a collection of text trees `T`
is a set of mappings from the variables of `P` to subtrees (values) of
text trees of `T` such that substituting these variables by their
associated values produce matching text trees (or in order words text
trees in the satisfying set of the pattern).

For instance if `P` is
```
(Lambda
  (VariableList
    (Variable "$X")
    (Variable "$Y"))
 (Inheritance
    (Variable "$X")
    (Variable "$Y")))
```

and `T` is

```
(Implication
  (Predicate "P")
  (Predicate "Q"))
(Inheritance
  (Concept "A")
  (Concept "B"))
(Inheritance
  (Concept "A")
  (Concept "C"))
(Inheritance
  (Concept "D")
  (Concept "D"))
```

then the satisfying set of `P` over `T` is
```
(Inheritance
  (Concept "A")
  (Concept "B"))
(Inheritance
  (Concept "A")
  (Concept "C"))
(Inheritance
  (Concept "D")
  (Concept "D"))
```

and its valuation set `V` is
```
{(Variable "$X")->(Concept "A"), (Variable "$Y")->(Concept "B")}
{(Variable "$X")->(Concept "A"), (Variable "$Y")->(Concept "C")}
{(Variable "$X")->(Concept "D"), (Variable "$Y")->(Concept "D")}
```

#### Step 3: Determine Shallow Abstractions

A shallow abstraction of a valuation set `V` over a variable `X` is
any pattern that match the values associated to `X`. Possible patterns
are

1. Constant node, such as `(Concept "A")`
2. Variable node positioned in the definition of `P` after `X`, such
   as `(Variable "$Y")`. The reason it must be positioned after `X` is
   to avoid redundancy.
3. Lambda links, such as
```
(Lambda
  (VariableLink
    (Variable "$Z")
    (Variable "$W"))
  (Implication
    (Variable "$Z")
    (Variable "$W")))
```

For example for the valuation set `V` defined above over variable
`(Variable "$X")`, the shallow abstractions would be `(Concept "A")`,
`(Concept "D")` and `(Variable "$Y")`. The last one comes the fact
that in the last valuation of `V`, the value associaed to `(Variable
"$Y")` is equal to the value associated to `(Variable "$X")` as well,
`(Concept "D")`.

Likewise the shallow abstractions of `(Variable "$Y")` would be
`(Concept "B")`, `(Concept "C")` and `(Concept "D")`, without
`(Variable "$X")` because, in spite of having some value in common,
`(Concept "D")`, it is positioned before `(Variable "$Y")` in the
variable declaration of `P`.

Another example, if the valuation set is the following singleton
```
{(Variable "$X")->(Implication (Predicate "P") (Predicate "Q"))}
```
its shallow abstraction over its single variable `(Variable "$X")` is
```
(Lambda
  (VariableList
    (Variable "$Z")
    (Variable "$W"))
  (Implication
    (Variable "$Z")
    (Variable "$W")))
```
because it corresponds to a pattern matching its value.

### Step 4: Specialize with Shallow Abstractions

Given all shallow abstractions associated to a certain pattern `P`
over all its variables, we can compose `P` with each of them to
produce specializations. For instance reusing `P`, `T` and `V` as
defined in the section detailing Step 2, the shallow abstractions over
variable `$(Variable "$X")` are `(Concept "A")`, `(Concept "D")` and
`(Variable "$Y")`. Likewise the shallow abstractions over variable
`$(Variable "$Y")` are `(Concept "B")`, `(Concept "C")` and `(Concept
"D")`.

To carry out the composition `Put` is used as follows
```
(Put
  P
  (List
    (Concept "A")
    (Variable "$Y")))
(Put
  P
  (List
    (Concept "D")
    (Variable "$Y")))
(Put
  P
  (List
    (Variable "$Y")
    (Variable "$Y")))
```
to replace `(Variable "$X")` by `(Concept "A")`, `(Concept "D")` and
`(Variable "$Y")` in `P`, producing
```
(Lambda
  (Variable "$Y")
  (Inheritance
    (Concept "A")
    (Variable "$Y")))
(Lambda
  (Variable "$Y")
  (Inheritance
    (Concept "D")
    (Variable "$Y")))
(Lambda
  (Variable "$Y")
  (Inheritance
    (Variable "$Y")
    (Variable "$Y")))
```

Then
```
(Put
  P
  (List
    (Variable "$X")
    (Concept "B")))
(Put
  P
  (List
    (Variable "$X")
    (Concept "C")))
(Put
  P
  (List
    (Variable "$X")
    (Concept "D")))
```

to replace `(Variable "$Y")` by `(Concept "B")`, `(Concept "C")` and
`(Concept "D")` in `P`, producing
```
(Lambda
  (Variable "$X")
  (Inheritance
    (Variable "$X")
    (Concept "B")))
(Lambda
  (Variable "$X")
  (Inheritance
    (Variable "$X")
    (Concept "C")))
(Lambda
  (Variable "$X")
  (Inheritance
    (Variable "$X")
    (Concept "D")))
```

### Step 5: Add Resulting Specializations with Enough Support

Given all specializations (6 in total in this iteration example), we
now need to calculate the frequency of each of them against `T`, and
only the one reaching the minimum support can be added back to the
population of patterns `C`. Out of these 6 only one has enough support
```
(Lambda
  (Variable "$Y")
  (Inheritance
    (Concept "A")
    (Variable "$Y")))
```

The others can be safely discarded because, according to the a priori
property, none of their subsequent specializations will reach the
minumum support.

One may notice that already in Step 4 we can avoid creating shallow
abstractions that we know will not lead to specializations with enough
support, just by counting the number of valuations matching a shallow
abstraction. This is used in the forward URE implementation as
explained in the subsection Enumerating Specializations with Forward
Chaining.

### Unified Rule Engine Implemenation

#### Motivation

Let us first explain why there is an incentive to implement such
algorithm in the URE, beside the coding simplifications that it may
offer. Pattern mining is a hard problem, not the hardest but still
hard, probably NP hard or something. Thus cleverly searching the space
of patterns is important. There are 2 places where careful decisions
matter, in Step 1, selecting the next pattern to specialize, and in
Step 4, selecting the shallow abstractions to be specialized
with. This highly resembles 2 steps of the URE algorithm, selecting
the next inference tree to expand, and selecting which node and rule
to expand it with. Since the URE has convenient inference control
mechanisms, ameable to self-learning, by framing the problem of
pattern mining onto the URE we inherit those control mechanisms.

#### Forward or Backward?

There are at least two ways to implement this algorithm in the URE, a
way which is more amebale to barckward chaining and another one more
amenable to forward chaining. The current implementation uses forward
chaining but both ways are presented here.

##### Enumerating Specializations with Backward Chaining

The backward chaining way would be to look for proofs of the theorem
that some initial pattern have some minimum support based on the
minimum support of its specializations. In other words, given the
following target to prove
```
minsup(P, T, ms)
```
where `P` is typically an abstract pattern, possibly the Top pattern,
`T` the texts in consideration, `ms` the value of minimum support
parameter, and `minsup` a predicate asserting that `P` has minimum
support `ms`.

There would be two main inference rules, one based on the a priori
property
```
minsup(Ps, T, ms)
special(Ps, P)
|-
minsup(P, T, ms)
```
meaning that if `Ps` has enough support and is a specialization of
`P`, then `P` has enough support. And a second rule to evaluate by
direct calculation if some pattern has enough support
```
ms <= freq(P, T)
|-
minsup(P, T, ms)
```
necessary when a pattern can no longer be specialized, or simply to
interrupt further specialization.

My mere virtue of attempting to proof the target the URE would
produced inference trees containing specializations.

##### Enumerating Specializations with Forward Chaining

The forward chaining way starts with the initial pattern as source and
derive inferences with the rule
```
minsup(P, T, ms)
special_minsup(Ps, P, T, ms)
|-
minsup(Ps, T, ms)
```
where `special_minsup` is a predicate that not only indicate that `Ps`
is a specialization of `P` but that such specialization will have
enough support, which can be determined by looking at the valuation
set used to generate the shallow abstractions, and thus the subsequent
specializations, as mentioned earlier.

In practice the predicate `special_minsup(Ps, P, T, ms)` is replaced
by `shallow-abstraction`, supplemented with a rule that calculates all
shallow abstraction that would yield specializations with enough
support.

TODO

Reboot
======

There are 2 ways to invoke the URE pattern miner

1. cog-miner, it will set up a rule-base, configure it, generate a
   query for the URE, run it, return the results, and delete the
   rule-base.
2. Manually using the URE.

Scretch
=======

I see 2 ways go with a URE pattern miner

1. Forward chainer based
2. Backward chainer based

In both cases we need to be able to isolate subpatterns from
superpatterns so that they can be processed individually, either
specialized (forward way), or abstracted (backward way).

## Forward Chainer Based

When the forward chainer is used, we start from an abstract pattern
like

```
Lambda
  X
  X
```

being the most abstract pattern, as it matches the entire atomspace,
and we create more specialized versions. There are 2 ways
specializations can be done

1. composing subpatterns into bigger patterns
2. expanding patterns, replacing leaves by bottom up subtrees

The first way would be for instance

```
Compose
  Type "InheritanceLink"
  Concept "a"
  Lambda
    X
    X
```

which would be in fact equivalent to

```
Lambda
  X
  InheritanceLink
    Concept "a"
    X
```

The fictive link `Compose`, probably badly named [likely constructs
from category theory would provide the right terminology], would take
a pattern, `(Lambda X X)`, and plug it into a bigger pattern starting
with `InheritanceLink`.

The second way would expand `X` in `(Lambda X X)` by for instance
`(InheritanceLink (Concept "a") X)`, obtaining the same

```
Lambda
  X
  InheritanceLink
    Concept "a"
    X
```

The first way is easily ameable to the URE, but does require higher
order constructs, like our badly named `Compose` link.

## Backward Chainer Based

In the backward way, rules express rewriting a specialized pattern
into a more abstract one, such as

```
Lambda
  X
  InheritanceLink
    Concept "a"
    X
```

into

```
Lambda
  X
  X
```

The case is so simple that we can imagine it's not difficult to write
a rule that does that. The problem comes when we want to process
larger patterns such as

```
Lambda
  X
  List
    InheritanceLink
      Concept "a"
      X
    InheritanceLink
      Concept "b"
      X
```

In order to have finite set of URE rules, we need to decompose the
pattern abstraction as to process the subtrees, such as

```
Inheritance
  Concept "a"
  X
```

as fully defined patterns, like

```
Lambda
  X
  Inheritance
    Concept "a"
    X
```

as opposed to a half-defined pattern like above it.

### ComposeLink

It seems that using ComposeLink can help. Let's consider the inference
control example

```
  ImplicationScope <rule-TV>
    VariableList
      Variable "$T"
      TypedVariable
        Variable "$A"
        Type "BindLink"
      Variable "$L"
      TypedVariable
        Variable "$B"
        Type "BindLink"
    And
      Execution
        GroundedSchema "expand-and-BIT"
        List
          Variable "$A"
          Variable "$L"
          <rule>
        Variable "$B"
      Evaluation
        Predicate "preproof-of"
        List
          Variable "$A"
          Variable "$T"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$B"
        Variable "$T"
```

We can then write the query

```
Compose
  ImplicationScope <rule-TV>
    VariableList
      Variable "$T"
      TypedVariable
        Variable "$A"
        Type "BindLink"
      Variable "$L"
      TypedVariable
        Variable "$B"
        Type "BindLink"
    And
      Execution
        GroundedSchema "expand-and-BIT"
        List
          Variable "$A"
          Variable "$L"
          <rule>
        Variable "$B"
      Evaluation
        Predicate "preproof-of"
        List
          Variable "$A"
          Variable "$T"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$B"
        Variable "$T"
  List
    Project 0
    Project 1
    Variable "$L-specialized"
    Project 3
```

where `Variable "$L-specialized"` is the variable of the BC query, and
`Project i` is a function projection constructor to let the (i+1)th
variable unchanged. A possible answer would be

```
Compose
  ImplicationScope <rule-TV>
    VariableList
      Variable "$T"
      TypedVariable
        Variable "$A"
        Type "BindLink"
      Variable "$L"
      TypedVariable
        Variable "$B"
        Type "BindLink"
    And
      Execution
        GroundedSchema "expand-and-BIT"
        List
          Variable "$A"
          Variable "$L"
          <rule>
        Variable "$B"
      Evaluation
        Predicate "preproof-of"
        List
          Variable "$A"
          Variable "$T"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$B"
        Variable "$T"
  List
    Project 0
    Project 1
    Lambda
      VariableList
        Variable "$T"
        TypedVariable
          Variable "$A"
          Type "BindLink"
        Variable "$X"
        TypedVariable
          Variable "$B"
          Type "BindLink"
      Inheritance
        Concept "a"
        Variable "$X"
    Project 3
```

Which would correspond to

```
ImplicationScope <rule-TV>
  VariableList
    Variable "$T"
    TypedVariable
      Variable "$A"
      Type "BindLink"
    Variable "$X"
    TypedVariable
      Variable "$B"
      Type "BindLink"
  And
    Execution
      GroundedSchema "expand-and-BIT"
      List
        Variable "$A"
        Inheritance
          Concept "a"
          Variable "$X"
        <rule>
      Variable "$B"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$A"
        Variable "$T"
  Evaluation
    Predicate "preproof-of"
    List
      Variable "$B"
      Variable "$T"
```

References
----------

[1] Yun Chi et al. Frequent Subtree Mining -- An Overview. (2005)
