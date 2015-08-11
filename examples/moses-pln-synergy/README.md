# MOSES PLN synergy demo

Simple demo showing how PLN reasoning can be used to overcome the
uncertainties resulting from the low number of samples during
learning.

## Run MOSES with the following command

```
$ moses \
      --input-file dataset.csv \
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
scheme@(guile-user)> (cog-bind pln-rule-for-all-hack)
scheme@(guile-user)> (cog-bind pln-rule-eliminate-neutral-element-hack)
scheme@(guile-user)> (cog-bind pln-rule-eliminate-dangling-junctor-hack)
scheme@(guile-user)> (cog-bind pln-rule-equivalence-hack)
$6 = (SetLink
   (ImplicationLink (stv 1 0.99999982)
      (PredicateNode "take-treatment-1" (stv 0.1 0.80000001))
      (PredicateNode "take-compound-A" (stv 0.2 0.80000001))
   )
)

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
