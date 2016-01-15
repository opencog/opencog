# MOSES PLN synergy demo

Simple demo showing how PLN reasoning can be used to overcome the
uncertainties resulting from the low number of samples during
learning.

## Run MOSES with the following command

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

## Load the models and the background knowledge in guile

`moses-model.scm` is actually provided containing the model (with
indentation + confidence). The confidence over the model is completely
made up, but it doesn't matter, we just want to convey the idea that
the use of background knowledge can improve that confidence.

```
$ guile
scheme@(guile-user)> (load "moses-model.scm")
scheme@(guile-user)> (load "background-knowledge.scm")
```

## Load the rule-based system (here PLN)

```scheme
scheme@(guile-user)> (load "pln-config.scm")
```

## Apply rules iteratively

### (1) - Partially instantiate if X takes Y and Y contains Z, then X takes Z, with Y = treatment-1 and Z = compound-A

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
   (ImplicationLink (stv 1 0.99999982)
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

### (2) - Distribute the lambda in the implicant and implicand of (1)

Semi-formally
```
(\x take(x, treatment-1) and contain(treatment-1, compound-A))
-> (\x take(x, compound-A))
```

```scheme
scheme@(guile-user)> (cog-bind implication-lambda-distribution-rule)
...
   (ImplicationLink (stv 1 0.99999982)
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
            (EvaluationLink (stv 1 0.99999982)
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

### (3) - Distribute the lambda in the conjunction of the implicant of (2)

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
         (EvaluationLink (stv 1 0.99999982)
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

### (4) - Calculate the TV of the constant predicate obtained in (3)

Semi-formally
```
\x contain(treatment-1, compound-A)
```

```scheme
scheme@(guile-user)> (cog-bind lambda-grounded-construction-rule)
$5 = (SetLink
   (LambdaLink (stv 1 0.99999982)
      (TypedVariableLink
         (VariableNode "$X")
         (TypeNode "ConceptNode")
      )
      (EvaluationLink (stv 1 0.99999982)
         (PredicateNode "contain")
         (ListLink
            (ConceptNode "treatment-1")
            (ConceptNode "compound-A")
         )
      )
   )
)
```

### (5) - Build the tautology that if X takes treatment-1, then treatment-1 contains compound-A

Semi-formally
```
(\x take(x, treatment-1)) -> (\x contain(treatment-1, compound-A))
```

```scheme
scheme@(guile-user)> (cog-bind implication-construction-rule)
...
   (ImplicationLink (stv 1 0.99999982)
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
      (LambdaLink (stv 1 0.99999982)
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (EvaluationLink (stv 1 0.99999982)
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

### (6) - Distribute the implicant in the implication (5)

Semi-formally
```
(\x take(x, treatment-1))
-> (\x take(x, treatment-1)) and (\x contain(treatment-1, compound-A))
```

```scheme
scheme@(guile-user)> (cog-bind implication-implicant-distribution-rule)
...
   (ImplicationLink (stv 1 0.99999982)
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
         (LambdaLink (stv 1 0.99999982)
            (TypedVariableLink
               (VariableNode "$X")
               (TypeNode "ConceptNode")
            )
            (EvaluationLink (stv 1 0.99999982)
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

### (7) - Factorize the lambda in the implicand of (6)

Semi-formally
```
(\x take(x, treatment-1)) and (\x contain(treatment-1, compound-A))
-> (\x (take(x, treatment-1) and contain(treatment-1, compound-A)))
```

```scheme
scheme@(guile-user)> (cog-bind implication-and-lambda-factorization-rule)
...
   (ImplicationLink (stv 1 0.99999982)
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
         (LambdaLink (stv 1 0.99999982)
            (TypedVariableLink
               (VariableNode "$X")
               (TypeNode "ConceptNode")
            )
            (EvaluationLink (stv 1 0.99999982)
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
            (EvaluationLink (stv 1 0.99999982)
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

### (8) - Using (6) and (7) deduce that if X takes treatment-1 then X takes treatment-1 and treatment-1 contains compound-A

Semi-formally
```
(\x take(x, treatment-1))
-> (\x (take(x, treatment-1) and contain(treatment-1, compound-A)))
```

```scheme
scheme@(guile-user)> (cog-bind deduction-implication-rule)
...
   (ImplicationLink (stv 1 0.99999982)
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
            (EvaluationLink (stv 1 0.99999982)
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

### (9) - Using (2) and (8) deduce that if X takes treatment-1 then X takes compound-A

Semi-formally

```
(\x takes(x, treatment-1))
-> (\x takes(x, compound-A))
```

```scheme
scheme@(guile-user)> ;; Actually the previous deduction step took care of that too
scheme@(guile-user)> ;; (cog-bind deduction-implication-rule)
...
   (ImplicationLink (stv 1 0.99999982)
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

### (10) - Fully instantiate that if a predicate is in the injury-recovery-speed-predicates class, then is-well-hydrated implies it

```scheme
scheme@(guile-user)> (cog-bind implication-full-instantiation-rule)
...
   (ImplicationLink (stv 0.69999999 0.60000002)
      (PredicateNode "is-well-hydrated")
      (PredicateNode "recovery-speed-of-injury-alpha")
   )
...
```

### (11) - Turn equivalence between `\x take(x, treatment-1)` and `take-treatment-1` into implications

```scheme
scheme@(guile-user)> (cog-bind equivalence-to-double-implication-rule)
...
      (ImplicationLink (stv 1 0.99999982)
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
      (ImplicationLink (stv 1 0.99999982)
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

### (12) - Use (11) and (9) to deduce that take-treatment-1 implies taking compound-A

Semi-formally
```
take-treatment-1 -> \x take(x, compound-A)
```

```scheme
scheme@(guile-user)> (cog-bind deduction-implication-rule)
...
   (ImplicationLink (stv 1 0.99999982)
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

### (13) - Use (12) and (11) to deduce `take-treatment-1 -> take-compound-A`

```scheme
scheme@(guile-user)> (cog-bind deduction-implication-rule)
...
   (ImplicationLink (stv 1 0.99999982)
      (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
      (PredicateNode "take-compound-A" (stv 0.2 0.80000001))
   )
...
```

### (14) - Use (13) and background knowledge that taking compound-A tends to speed-up recovery of injury alpha to deduce that take-treatment-1 implies recovery-speed-of-injury-alpha

Semi-formally
```
take-treatment-1 -> recovery-speed-of-injury-alpha
```

```scheme
scheme@(guile-user)> (cog-bind deduction-implication-rule)
```
   (ImplicationLink (stv 0.55000001 0.89999998)
      (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
      (PredicateNode "recovery-speed-of-injury-alpha" (stv 0.30000001 0.80000001))
   )
```
```

### (15) - Use (10) and the background knowledge that eating a lot of fruits and vegetables hydrates well to deduce that eat-lost-fruits-vegetables implies recovery-speed-of-injury-alpha

Semi-formally
```
eat-lost-fruits-vegetables -> recovery-speed-of-injury-alpha
```

TODO: can't infer that at all

```scheme
scheme@(guile-user)> (cog-bind deduction-implication-rule)
```
   (ImplicationLink ???
      (PredicateNode "eat-lots-fruits-vegetables" (stv 0.07 0.8))
      (PredicateNode "recovery-speed-of-injury-alpha" (stv 0.30000001 0.80000001))
   )
```
```

### (16) - Using (15) and (14) with the implication-or rule

```
$10 = (SetLink
   (ImplicationLink (stv 0.6447677 0.60000002)
      (OrLink
         (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
         (PredicateNode "eat-lots-fruits-vegetables" (stv 0.07 0.80000001))
      )
      (PredicateNode "recovery-speed-of-injury-alpha" (stv 0.80000001 0))
   )
...
```
