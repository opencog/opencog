# MOSES PLN synergy demo

Simple demo showing how PLN reasoning can be used to overcome the
uncertainties resulting from the low number of samples during
learning.

## Run MOSES with the following command

```
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

```
scheme@(guile-user)> (load "pln-config.scm")
```

## Apply rules iteratively

```
scheme@(guile-user)> ;; Infer that take-treatment-1 implies take-compound-A
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
scheme@(guile-user)> ;; If X takes treatment-1 then X takes compound-A
scheme@(guile-user)> (cog-bind deduction-implication-rule)
...
   (ImplicationLink (stv 0 0.80000001)
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
scheme@(guile-user)> ;; Infer that being well hydrated speeds up recovery 
scheme@(guile-user)> (cog-bind pln-rule-average-hack)
scheme@(guile-user)> (cog-bind pln-rule-modus-ponens)
$8 = (SetLink
   ...
   (ImplicationLink (stv 0.69999999 0.60000002)
      (PredicateNode "is-well-hydrated" (stv 0.059500001 0.80000001))
      (PredicateNode "recovery-speed-of-injury-alpha" (stv 0.80000001 0))
   )
)

scheme@(guile-user)> ;; Infer relationships between input and target features
scheme@(guile-user)> (cog-bind pln-rule-deduction)
$5 = (SetLink
   (ImplicationLink (stv 0.55000001 0.80000001)
      (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
      (PredicateNode "recovery-speed-of-injury-alpha" (stv 0.875 0))
   )
   ...
   (ImplicationLink (stv 0.7279107 0.60000002)
      (PredicateNode "eat-lots-fruits-vegetables" (stv 0.07 0.80000001))
      (PredicateNode "recovery-speed-of-injury-alpha" (stv 0.875 0))
   )
)

scheme@(guile-user)> ;; Infer MOSES model precision
scheme@(guile-user)> (cog-bind pln-rule-implication-or)
$10 = (SetLink
   (ImplicationLink (stv 0.6447677 0.60000002)
      (OrLink
         (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
         (PredicateNode "eat-lots-fruits-vegetables" (stv 0.07 0.80000001))
      )
      (PredicateNode "recovery-speed-of-injury-alpha" (stv 0.80000001 0))
   )
   ...
