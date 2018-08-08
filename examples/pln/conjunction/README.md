Conjunction
===========

Simple example showing how to calculate the TV of a conjunction using
the backward chainer.

Usage
-----

Load the conjunction rule base in the guile interpreter:

```bash
$ guile -l conjunction-rule-base-config.scm
```

Optionally, set the log level to debug (can be insightful to read):

```scheme
(ure-logger-set-level! "debug")
```

Define a conjunction to evaluate:

```scheme
(define P (Predicate "P"))
(define A (Concept "A"))
(define B (Concept "B"))
(define PA (Evaluation (stv 0.5 0.8) P A))
(define PB (Evaluation (stv 0.3 0.9) P B))
(define PAPB (And PA PB))
```

As you see simple TVs have been assigned to PA and PB but not to its
conjunction. By default a simple TV with null confidence (equivalent
to total ignorance) is assigned. Now let's call the backward chainer
on the conjunction to evaluate its TV:

```scheme
(conj-bc PAPB)
```

You should get something

```scheme
$2 = (SetLink
   (AndLink (stv 0.3 0.8)
      (EvaluationLink (stv 0.3 0.9)
         (PredicateNode "P")
         (ConceptNode "B")
      )
      (EvaluationLink (stv 0.5 0.8)
         (PredicateNode "P")
         (ConceptNode "A")
      )
   )
)
```
