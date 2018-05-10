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

By mere virtue of attempting to proof the target the URE would
produced inference trees containing specializations. This however
requires to either check the a priori property in the inference
control to make sure specializations that do not have enough support
are not being built for no reason while inference trees are being
expanded backward.

##### Enumerating Specializations with Forward Chaining

The forward chaining way starts with the initial pattern as source and
derive inferences with the rule
```
minsup(P, T, ms)
shallow-abstraction(S, P, T, ms)
|-
minsup((Put P S), T, ms)
```
where `shallow-abstraction` is a predicate that not only `S` is a
shallow abstraction of `P` over `T` but also if `P` is composed with
such, the resulting pattern will reach minimum support, which can
indeed be determined by looking at the valuation set used to generate
the shallow abstractions, as mentioned earlier.

The advantage of the forward technique is that it requires no control
for the a priori property as it is built into the rule.

In the end we have 2 rules, in the `rules` folders

1. Shallow abstraction rule, to produce all shallow abstractions of
   given pattern, texts and minimum support, over all its variables.
2. Specialization rule, to produce specializations by composition a
   pattern with its shallow abstractions.

The shallow abstraction rule is actually implemented in C++, for
effeciency reason and produce in one go all shallow abstractions
reaching the minimum support. For that reason, there is only one
decision that really matters in the pattern miner control, the one in
Step 1: selecting the next pattern to specialize.

#### Minimum Support Formalization

Knowledge such as `minsup(P, T, ms)` is simply being formalized as
```
Evaluation (stv 1 1)
  Predicate "minsup"
  List
    P
    T
    ms
```

where `P` is the pattern, `T` is a concept node of the text set, so
that each element of it is a text, and `ms` is a number node with the
minimum support parameter.

When the pattern miner is invoked the discovered patterns will be
delivered according to that format.

Usage
-----

To invoke the pattern miner, within guile, you first need to import
the `miner` module
```
(use-modules (opencog miner))
```

Then, simply call `cog-miner` with your text set and minimum
support. The text set can be either

1. a Scheme list of atoms
2. an Atomese List or Set of atoms
3. an atomspace (use (cog-atomspace) to get the current one)
4. a concept node such that all its members are texts

TODO: give example.

References
----------

[1] Yun Chi et al. Frequent Subtree Mining -- An Overview. (2005)
