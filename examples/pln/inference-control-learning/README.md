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
small knowledge-base. No ECAN is even necessary. The learned inference
control rule should be that double deduction is useful.

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
Cognitive Schematic, context free for starter.

Implication
  ExecutionOutput
  

TODO: give inference control rule examples

  Context:
    running the backward chainer to prove T
    about to choose what rule to expand L of and-BIT A
  Action:
    expand with rule R
  Result:
    produce a proof-suffix of T

TODO: A proof-suffix is a inference tree prefix of an inference tree proving T

;; Top inference control rule. This is the most abstract inference control rule
;; expressing the probability of that expanding and-BIT will produce an and-BIT
;; that is a proof-suffix of any target. This is good for instance to evaluate
;; the performance of a BC policy. All other inference control rule are
;; specializations of this rule.

ImplicationScope
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

