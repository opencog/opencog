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

The useful to the Backward Chainer format should be that of a context
free Cognitive Schematic.

TODO: give inference control rule examples
