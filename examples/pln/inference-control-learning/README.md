Inference Control Learning
==========================

Experiment with inference control learning. The plan is to

1. run a series of backward inferences,
2. create a corpus of successful ones,
3. Run the pattern miner over them,
4. build cognitive schematics out of these patterns,
5. repeat with more inferences passing these cognitive schematics to
   the backward chainer for speeding it up.

For now it is a toy experiment with a small taylored series over a
small, constant knowledge-base. No ECAN is even necessary. The learned
inference control rule should be that double deduction is useful.

Knowledge-base
--------------

The knowledge-base is the upper case latin alphabet order. The order
relationship is represented with Inheritance between 2 consecutive
letters:

Inheritance (stv 1 1)
  Concept "A" (stv 1/26 1)
  Concept "B" (stv 2/26 1)

...

Inheritance (stv 1 1)
  Concept "Y" (stv 25/26 1)
  Concept "Z" (stv 26/26 1)

The concept strengths, 1/26 to 26/26, are defined to be consistent
with PLN semantics.

Rule-base
---------

All PLN rules, to make finding proofs more difficult.

Targets
-------

The targets to prove are of the form

Inheritance
  X
  Y

where X is a letter preceding Y.

Inference Control Rule
----------------------

The inference control rules should be that double, triple, etc,
deductions are the most effective way to prove that 2 letters are
ordered.

The useful representation for the Backward Chainer should be a
Cognitive Schematic, where

* Context:
  + Running the backward chainer to prove T
  + About to choose what rule to expand and-BIT A from leaf L
  + Fulfill some pattern
* Action:
  + Expand with rule R
* Result:
  + Produce a proof-suffix of T

TODO: A proof-suffix is an inference subtree (in the graph theory
sense) of an inference tree proving T.

Typically the context that we are running the backward chainer to
prove T and about to decide what rule to expand with will be assumed,
because that will be the universal context of our experiment anyway.

The simplest possible inference control rule that we can learn, let's
call it level-0 meta-learning, expresses the probability of expanding
an and-BIT producing a proof-suffix, or "subproof", of any target. All
other inference control rules are specializations of this rule.

ImplicationScope <TV>
  VariableList
    Variable "$T"  ;; Theorem/target to prove
    TypedVariable  ;; and-BIT to expand
      Variable "$A"
      Type "BindLink"
    Variable "$L"  ;; Leaf from A to expand
    Variable "$R"  ;; Rule to expand A from L
    TypedVariable  ;; Resulting and-BIT from the expansion of L from A with rule R
      Variable "$B"
      Type "BindLink"
  Execution
    GroundedSchema "expand-and-BIT"
    List
      "$A"
      "$L"
      "$R"
    "$B"
  Evaluation
    Predicate "subproof"
    List
      Variable "$B"
      Variable "$T"

Such an inference control rule may already help a bit the Backward
Chainer. Indeed partial instantiation over R will calculate the
probability of a given rule to produce a subproof of any T. To obtain
a partial instantiation over R, R is subtituted by a certain rule,
called <rule>, and the TV over the ImplicationScope, called <rule-TV>,
is obtained by direct evaluation over the subset of observations of
the corpus of proofs involving <rule>.

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
  Execution
    GroundedSchema "expand-and-BIT"
    List
      "$A"
      "$L"
      <rule>
    "$B"
  Evaluation
    Predicate "subproof"
    List
      Variable "$B"
      Variable "$T"

This is a good way to assign the weights of the rules for the next
round of reasoning, but it might not go much further. Indeed partial
instantiation will likely fail for unknown instances of T, A, L or R
because direct evaluation won't be possible. If at least some
instances have been encountered, for instance T and A are known but L
isn't, then we can still come up with some possibly useful partial
instantiation of that rule, such as

ImplicationScope <target-andbit-rule-TV>
  VariableList
    Variable "$L"
    TypedVariable
      Variable "$B"
      Type "BindLink"
  Execution
    GroundedSchema "expand-and-BIT"
    List
      <andbit>
      "$L"
      <rule>
    "$B"
  Evaluation
    Predicate "subproof"
    List
      Variable "$B"
      <target>

where <target-andbit-rule-TV> is calculated based on the subset of
observations in the corpus where <target>, <andbit> and <rule> occurs
simultanously. In most cases though such subset will be small and
<target-andbit-rule-TV> will have a low confidence. Moreover the
complexity of such a predictor will be high because the complexity of
the instances of T, A, etc, will be counted as well, which will give
it a low prior and in turn will lower the confidence of the produced
term by conditional instantiation (see Section Conditional
Instantiation Confidence).

To go beyond this level-0 meta-learning type we need to introduce more
expressive specializations in order to generalize well when new
instances of T, A, L, etc, are encountered.

Let's give the general form of such specialization

ImplicationScope <TV>
  VariableList
    Variable "$T"  ;; Theorem/target to prove
    TypedVariable  ;; and-BIT to expand
      Variable "$A"
      Type "BindLink"
    Variable "$L"  ;; Leaf from A to expand
    Variable "$R"  ;; Rule to expand A from L
    TypedVariable  ;; Resulting and-BIT from the expansion of L from A with rule R
      Variable "$B"
      Type "BindLink"
  And
    Execution
      GroundedSchema "expand-and-BIT"
      List
        "$A"
        "$L"
        "$R"
      "$B"
    <pattern>
  Evaluation
    Predicate "subproof"
    List
      Variable "$B"
      Variable "$T"

where <pattern> is a predicate body involving all or some of the
variables T, A, L and R.

TODO: give an example with double deduction.

  (ExecutionOutputLink
    (GroundedSchemaNode "scm: bc-deduction-formula") ; [5481509939359570705][1]
    (ListLink
      (InheritanceLink
        (ConceptNode "A") ; [6977206836600634430][1]
        (ConceptNode "D") ; [246112806454457922][1]
      ) ; [9592798904207778024][1]
      (ExecutionOutputLink
        (GroundedSchemaNode "scm: bc-deduction-formula") ; [5481509939359570705][1]
        (ListLink
          (InheritanceLink
            (ConceptNode "A") ; [6977206836600634430][1]
            (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
          ) ; [13444058388333684400][15]
          (InheritanceLink
            (ConceptNode "A") ; [6977206836600634430][1]
            (VariableNode "$B-6229393a") ; [6185394777777469381][15]
          ) ; [15532080875530789483][15]
          (InheritanceLink
            (VariableNode "$B-6229393a") ; [6185394777777469381][15]
            (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
          ) ; [14984376557733565207][15]
        ) ; [18175419943537721797][15]
      ) ; [16724333062125429183][15]
      (InheritanceLink
        (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
        (ConceptNode "D") ; [246112806454457922][1]
      ) ; [16015351290941397556][15]
    ) ; [15046555205905734382][15]
  ) ; [13595468324493441768][15]

[15532080875530789483][15] [14984376557733565207][15]
----------------bc-deduction-formula-----------------
             [13444058388333684400][15] [16015351290941397556][15]
             ----------------bc-deduction-formula-----------------
                           [9592798904207778024][1]

[DEBUG] [URE] Expanded forward chainer strategy:
(BindLink
  (VariableList
    (TypedVariableLink
      (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
      (TypeNode "ConceptNode") ; [3788634541270868382][1]
    ) ; [11809658565253834475][15]
    (TypedVariableLink
      (VariableNode "$B-6229393a") ; [6185394777777469381][15]
      (TypeNode "ConceptNode") ; [3788634541270868382][1]
    ) ; [16150796384774871558][15]
  ) ; [10107478639882222213][15]
  (AndLink
  ) ; [17473895290224991152][15]
  (ExecutionOutputLink
    (GroundedSchemaNode "scm: bc-deduction-formula") ; [5481509939359570705][1]
    (ListLink
      (InheritanceLink
        (ConceptNode "A") ; [6977206836600634430][1]
        (ConceptNode "D") ; [246112806454457922][1]
      ) ; [9592798904207778024][1]
      (ExecutionOutputLink
        (GroundedSchemaNode "scm: bc-deduction-formula") ; [5481509939359570705][1]
        (ListLink
          (InheritanceLink
            (ConceptNode "A") ; [6977206836600634430][1]
            (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
          ) ; [13444058388333684400][15]
          (InheritanceLink
            (ConceptNode "A") ; [6977206836600634430][1]
            (VariableNode "$B-6229393a") ; [6185394777777469381][15]
          ) ; [15532080875530789483][15]
          (InheritanceLink
            (VariableNode "$B-6229393a") ; [6185394777777469381][15]
            (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
          ) ; [14984376557733565207][15]
        ) ; [18175419943537721797][15]
      ) ; [16724333062125429183][15]
      (InheritanceLink
        (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
        (ConceptNode "D") ; [246112806454457922][1]
      ) ; [16015351290941397556][15]
    ) ; [15046555205905734382][15]
  ) ; [13595468324493441768][15]
) ; [17301981865595416016][15]

[DEBUG] [URE] With inference tree:

Conditional Instantiation Confidence
------------------------------------

TODO: look into universal operator induction

P(D') = sum_M P(D'|M) * P(M|D) * P(D)
