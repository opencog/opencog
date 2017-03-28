# MOSES PLN synergy demo

Simple demo showing how PLN reasoning can be used to overcome the
uncertainties resulting from the low number of samples during
learning.

There are 3 ways for carrying the inference

1. using the pattern matcher in a step-by-step fashion, see Subsection
   `Pattern Matcher`
2. using the forward chainer, see Subsection `Forward Chainer`
3. using the backward chainer, not done yet.

## Obtain MOSES model

A MOSES has been pre-learned and included in the parent folder
`../moses-model.scm`. The confidence over the model has been completely
made up, but it doesn't matter, we just want to convey the idea that
the use of background knowledge can improve that confidence.

Alternatively you may run the learning yourself

```bash
$ moses \
      --input-file ../dataset.csv \
      --target-feature recovery-speed-of-injury-alpha \
      --output-with-labels 1 \
      --problem pre \
      -q 0.7 \
      -p 0.5 \
      --result-count 1 \
      --output-format scheme
```

## Forward Chainer

To run the demo using the forward chainer load the following script

```
$ guile
scheme@(guile-user)> (load "moses-pln-synergy-fc.scm")
```

in guile. It will load the model, the background knowledge, the PLN
config file and run the inference. Beware that it's gonna take a
really long time (say 3 to 4 days depending on your hardware) as it
runs 1M of inference steps, and that each step combines many atoms
(one source + one rule + all possible other premises).

Once the inference finished you can check that the final target (the
MOSES model is infered with the correct TV

```scheme
(ImplicationLink (stv 0.60357851 0.69999999)
   (OrLink
      (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
      (PredicateNode "eat-lots-fruits-vegetables" (stv 0.07 0.80000001))
   )
   (PredicateNode "recovery-speed-of-injury-alpha" (stv 0.30000001 0.80000001))
)
```

to check the TV merely enter that hypergraph without the TVs. You may
of course check each step detailed in subsection `Step-by-step` below
as well, similarily by entering them without their TVs.

There are currently no method to query the trace of the inference but
this will be developed as time goes.

## Backward Chainer

To run the demo using the forward chainer load the following script

```
$ guile
scheme@(guile-user)> (load "moses-pln-synergy-bc.scm")
```

in guile. It will load the model, the background knowledge, the PLN
config file and run the inference. Beware that it's gonna take a
really long time (say 3 to 4 days depending on your hardware) as it
runs 1M of inference steps, and that each step combines many atoms
(one source + one rule + all possible other premises).

Once the inference finished you can check that the final target (the
MOSES model is infered with the correct TV

```scheme
(ImplicationLink (stv 0.60357851 0.69999999)
   (OrLink
      (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
      (PredicateNode "eat-lots-fruits-vegetables" (stv 0.07 0.80000001))
   )
   (PredicateNode "recovery-speed-of-injury-alpha" (stv 0.30000001 0.80000001))
)
```

to check the TV merely enter that hypergraph without the TVs. You may
of course check each step detailed in subsection `Step-by-step` below
as well, similarily by entering them without their TVs.

There are currently no method to query the trace of the inference but
this will be developed as time goes.

## Pattern Matcher

### Batch

You may choose to run the inference all at once. For that just load
`moses-pln-synergy-pm.scm` in guile

```
$ guile
scheme@(guile-user)> (load "moses-pln-synergy-pm.scm")
```

Which will load the MOSES model, the background knowledge, the PLN
configuration for that inference and execute all steps in a row (by
using the pattern matcher). If you wish to execute the steps yourself
read on.

### Step-by-step

Alternatively you may run it step by step. First you need to run guile
and load the models, the background knowledge and the PLN
configuration.

```
$ guile
scheme@(guile-user)> (load "moses-model.scm")
scheme@(guile-user)> (load "background-knowledge.scm")
scheme@(guile-user)> (load "pln-fc-config.scm")
```

#### Apply rules

Then apply the following rules step by step.

##### (1) - Partially instantiate if X takes Y and Y contains Z, then X takes Z, with Y = treatment-1 and Z = compound-A

Semi-formally
```
\x (take(x, treatment-1) and contain(treatment-1, compound-A))
-> take(x, compound-A)
```

```scheme
scheme@(guile-user)> (for-each (lambda (i) (cog-bind implication-partial-instantiation-rule)) (iota 2))
scheme@(guile-user)> (cog-prt-atomspace)
And search for the following
...
   (ImplicationScopeLink (stv 1 1)
      (TypedVariableLink
         (VariableNode "$X")
         (TypeNode "ConceptNode")
      )
      (AndLink
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (ConceptNode "treatment-1")
            )
         )
         (EvaluationLink (stv 1 0.99999982)
            (PredicateNode "contain")
            (ListLink
               (ConceptNode "treatment-1")
               (ConceptNode "compound-A")
            )
         )
      )
      (EvaluationLink
         (PredicateNode "take")
         (ListLink
            (VariableNode "$X")
            (ConceptNode "compound-A")
         )
      )
   )
...
```

##### (2) - Convert the implication scope of (1) into an implication

Semi-formally
```
(\x take(x, treatment-1) and contain(treatment-1, compound-A))
-> (\x take(x, compound-A))
```

```scheme
scheme@(guile-user)> (cog-bind implication-scope-to-implication-rule)
...
   (ImplicationLink (stv 1 1)
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (AndLink
            (EvaluationLink
               (PredicateNode "take")
               (ListLink
                  (VariableNode "$X")
                  (ConceptNode "treatment-1")
               )
            )
            (EvaluationLink (stv 1 1)
               (PredicateNode "contain")
               (ListLink
                  (ConceptNode "treatment-1")
                  (ConceptNode "compound-A")
               )
            )
         )
      )
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (ConceptNode "compound-A")
            )
         )
      )
   )
...
```

##### (3) - Distribute the lambda in the conjunction of the implicant of (2)

Semi-formally
```
(\x take(x, treatment-1)) and (\x contain(treatment-1, compound-A))
```

```scheme
scheme@(guile-user)> (cog-bind and-lambda-distribution-rule)
...
   (AndLink
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (ConceptNode "treatment-1")
            )
         )
      )
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (EvaluationLink (stv 1 1)
            (PredicateNode "contain")
            (ListLink
               (ConceptNode "treatment-1")
               (ConceptNode "compound-A")
            )
         )
      )
   )
...
```

##### (4) - Calculate the TV of the constant predicate obtained in (3)

Semi-formally
```
\x contain(treatment-1, compound-A)
```

```scheme
scheme@(guile-user)> (cog-bind closed-lambda-introduction-rule)
$5 = (SetLink
   (LambdaLink (stv 1 1)
      (TypedVariableLink
         (VariableNode "$X")
         (TypeNode "ConceptNode")
      )
      (EvaluationLink (stv 1 1)
         (PredicateNode "contain")
         (ListLink
            (ConceptNode "treatment-1")
            (ConceptNode "compound-A")
         )
      )
   )
)
```

##### (5) - Build the tautology using (4) that if X takes treatment-1, then treatment-1 contains compound-A

Semi-formally
```
(\x take(x, treatment-1)) -> (\x contain(treatment-1, compound-A))
```

```scheme
scheme@(guile-user)> (cog-bind implication-introduction-rule)
...
   (ImplicationLink (stv 1 1)
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (ConceptNode "treatment-1")
            )
         )
      )
      (LambdaLink (stv 1 1)
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (EvaluationLink (stv 1 1)
            (PredicateNode "contain")
            (ListLink
               (ConceptNode "treatment-1")
               (ConceptNode "compound-A")
            )
         )
      )
   )
...
```

##### (6) - Distribute the implicant in the implication (5)

Semi-formally
```
(\x take(x, treatment-1))
-> (\x take(x, treatment-1)) and (\x contain(treatment-1, compound-A))
```

```scheme
scheme@(guile-user)> (cog-bind implication-implicant-distribution-rule)
...
   (ImplicationLink (stv 1 1)
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (ConceptNode "treatment-1")
            )
         )
      )
      (AndLink
         (LambdaLink
            (TypedVariableLink
               (VariableNode "$X")
               (TypeNode "ConceptNode")
            )
            (EvaluationLink
               (PredicateNode "take")
               (ListLink
                  (VariableNode "$X")
                  (ConceptNode "treatment-1")
               )
            )
         )
         (LambdaLink (stv 1 1)
            (TypedVariableLink
               (VariableNode "$X")
               (TypeNode "ConceptNode")
            )
            (EvaluationLink (stv 1 1)
               (PredicateNode "contain")
               (ListLink
                  (ConceptNode "treatment-1")
                  (ConceptNode "compound-A")
               )
            )
         )
      )
   )
...
```

##### (7) - Factorize the lambda in the implicand of (6)

Semi-formally
```
(\x take(x, treatment-1)) and (\x contain(treatment-1, compound-A))
-> (\x (take(x, treatment-1) and contain(treatment-1, compound-A)))
```

```scheme
scheme@(guile-user)> (cog-bind implication-and-lambda-factorization-rule)
...
   (ImplicationLink (stv 1 1)
      (AndLink
         (LambdaLink
            (TypedVariableLink
               (VariableNode "$X")
               (TypeNode "ConceptNode")
            )
            (EvaluationLink
               (PredicateNode "take")
               (ListLink
                  (VariableNode "$X")
                  (ConceptNode "treatment-1")
               )
            )
         )
         (LambdaLink (stv 1 1)
            (TypedVariableLink
               (VariableNode "$X")
               (TypeNode "ConceptNode")
            )
            (EvaluationLink (stv 1 1)
               (PredicateNode "contain")
               (ListLink
                  (ConceptNode "treatment-1")
                  (ConceptNode "compound-A")
               )
            )
         )
      )
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (AndLink
            (EvaluationLink
               (PredicateNode "take")
               (ListLink
                  (VariableNode "$X")
                  (ConceptNode "treatment-1")
               )
            )
            (EvaluationLink (stv 1 1)
               (PredicateNode "contain")
               (ListLink
                  (ConceptNode "treatment-1")
                  (ConceptNode "compound-A")
               )
            )
         )
      )
   )
...
```

##### (8) - Using (6) and (7) deduce that if X takes treatment-1 then X takes treatment-1 and treatment-1 contains compound-A

Semi-formally
```
(\x take(x, treatment-1))
-> (\x (take(x, treatment-1) and contain(treatment-1, compound-A)))
```

```scheme
scheme@(guile-user)> (cog-bind deduction-implication-rule)
...
   (ImplicationLink (stv 1 1)
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (ConceptNode "treatment-1")
            )
         )
      )
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (AndLink
            (EvaluationLink
               (PredicateNode "take")
               (ListLink
                  (VariableNode "$X")
                  (ConceptNode "treatment-1")
               )
            )
            (EvaluationLink (stv 1 1)
               (PredicateNode "contain")
               (ListLink
                  (ConceptNode "treatment-1")
                  (ConceptNode "compound-A")
               )
            )
         )
      )
   )
...
```

##### (9) - Using (8) and (2) deduce that if X takes treatment-1 then X takes compound-A

Semi-formally

```
(\x takes(x, treatment-1))
-> (\x takes(x, compound-A))
```

```scheme
scheme@(guile-user)> (cog-bind deduction-implication-rule)
...
   (ImplicationLink (stv 1 1)
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (ConceptNode "treatment-1")
            )
         )
      )
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (ConceptNode "compound-A")
            )
         )
      )
   )
...
```

##### (10) - Fully instantiate that if a predicate is in the injury-recovery-speed-predicates class, then is-well-hydrated implies it

```scheme
scheme@(guile-user)> (cog-bind implication-full-instantiation-rule)
...
   (ImplicationLink (stv 0.69999999 0.52499998)
      (PredicateNode "is-well-hydrated")
      (PredicateNode "recovery-speed-of-injury-alpha")
   )
...
```

##### (11) - Turn equivalences such as between `\x take(x, treatment-1)` and `take-treatment-1` into implications

```scheme
scheme@(guile-user)> (cog-bind equivalence-to-implication-rule)
...
      (ImplicationLink (stv 1 1)
         (LambdaLink
            (TypedVariableLink
               (VariableNode "$X")
               (TypeNode "ConceptNode")
            )
            (EvaluationLink
               (PredicateNode "take")
               (ListLink
                  (VariableNode "$X")
                  (ConceptNode "compound-A")
               )
            )
         )
         (PredicateNode "take-compound-A" (stv 0.2 0.80000001))
      )
...
      (ImplicationLink (stv 1 1)
         (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
         (LambdaLink
            (TypedVariableLink
               (VariableNode "$X")
               (TypeNode "ConceptNode")
            )
            (EvaluationLink
               (PredicateNode "take")
               (ListLink
                  (VariableNode "$X")
                  (ConceptNode "treatment-1")
               )
            )
         )
      )
...
```

##### (12) - Use (11) and (9) to deduce that take-treatment-1 implies taking compound-A

Semi-formally
```
take-treatment-1 -> \x take(x, compound-A)
```

```scheme
scheme@(guile-user)> (cog-bind deduction-implication-rule)
...
   (ImplicationLink (stv 1 1)
      (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (ConceptNode "compound-A")
            )
         )
      )
   )
...
```

##### (13) - Use (12) and (11) to deduce `take-treatment-1 -> take-compound-A`

```scheme
scheme@(guile-user)> (cog-bind deduction-implication-rule)
...
   (ImplicationLink (stv 1 1)
      (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
      (PredicateNode "take-compound-A" (stv 0.2 0.80000001))
   )
...
```

##### (14) - Use (13) and background knowledge that taking compound-A tends to speed-up recovery of injury alpha to deduce that take-treatment-1 implies recovery-speed-of-injury-alpha

Semi-formally
```
take-treatment-1 -> recovery-speed-of-injury-alpha
```

```scheme
scheme@(guile-user)> (cog-bind deduction-implication-rule)
...
   (ImplicationLink (stv 0.55000001 0.81)
      (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
      (PredicateNode "recovery-speed-of-injury-alpha" (stv 0.30000001 0.80000001))
   )
...
```

##### (15) - Use (10) and the background knowledge that eating a lot of fruits and vegetables hydrates well to deduce that eat-lost-fruits-vegetables implies recovery-speed-of-injury-alpha

Semi-formally
```
eat-lost-fruits-vegetables -> recovery-speed-of-injury-alpha
```

```scheme
scheme@(guile-user)> (cog-bind deduction-implication-rule)
...
   (ImplicationLink (stv 0.62 0.64124995)
      (PredicateNode "eat-lots-fruits-vegetables" (stv 0.07 0.80000001))
      (PredicateNode "recovery-speed-of-injury-alpha" (stv 0.30000001 0.80000001))
   )
...
```

##### (16) - Using (15) and (14) with the implication-implicant-disjunction rule we can infer a new TV for the MOSES model

```
scheme@(guile-user)> (cog-bind implication-implicant-disjunction-rule)
$10 = (SetLink
   (ImplicationLink (stv 0.60357851 0.64124995)
      (OrLink
         (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
         (PredicateNode "eat-lots-fruits-vegetables" (stv 0.07 0.80000001))
      )
      (PredicateNode "recovery-speed-of-injury-alpha" (stv 0.30000001 0.80000001))
   )
...
```

This is our MOSES model! By reasoning we managed to update its TV, the
strength is lower but most importantly its confidence is higher. In
other words, with reasoning we could make it up for a poor dataset.
