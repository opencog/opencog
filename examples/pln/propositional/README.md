Propositional
=============

Simple example showing how to calculate the TV of a fuzzy proposition
using the backward chainer.

Usage
-----

Load the propositional rule base in the guile interpreter:

```bash
$ guile -l propositional-rule-base-config.scm
```

Optionally, set the log level to debug (can be insightful to read):

```scheme
(use-modules (opencog logger))
(cog-logger-set-level! "debug")
```

Define a proposition to evaluate:

```scheme
(define P (Predicate "P"))
(define Q (Predicate "Q"))
(define A (Concept "A"))
(define B (Concept "B"))
(define C (Concept "C"))
(define PA (Evaluation (stv 0.5 0.8) P A))
(define PB (Evaluation (stv 0.3 0.9) P B))
(define QC (Evaluation (stv 0.9 0.7) Q C))
(define proposition (Or (And PA PB) (Not QC)))
```

As you see simple TVs have been assigned to PA, PB and QC but not to
the proposition. By default a simple TV with null confidence
(equivalent to total ignorance) is assigned. Now let's call the
backward chainer on the proposition to evaluate its TV:

```scheme
(prop-bc proposition)
```

You should get something

```scheme
$2 = (SetLink
   (OrLink (stv 0.3 0.7)
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
      (NotLink (stv 0.1 0.7)
         (EvaluationLink (stv 0.9 0.7)
            (PredicateNode "Q")
            (ConceptNode "C")
         )
      )
   )
)
```
