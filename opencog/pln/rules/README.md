# PLN Rule definitions for the URE

Implements PLN rules according to
http://wiki.opencog.org/w/URE_Configuration_Format

## Contents

PLN rules are partitioned into boolean, predicate and term logic.
These terms are used loosely since PLN is rather different than any of
those. The folders are organized as follows

- `boolean-logic` contains rules dealing with `AndLink`, `OrLink` and
  `NotLink` evaluation and introduction. They are fuzzy for the most
  part, contrary to what the name folder seems to suggest.
- `predicate-logic` contains rules for universal and conditional
  instantiation and generalization.
- `term-logic` contains rules dealing with deduction, abduction,
  induction and inversion.
- `wip` contains work-in-progress rules, incomplete or broken.

## PLN Examples

Multiple examples can be found under

```
<OPENCOG_REPO>/examples/pln
```

## Relex2Logic Example

The r2l subdirectory has the initial specification of the
R2L-RuleBase for the English language.

   ```
   opencog/nlp/relex2logic/rules
   ```

To test the rules, use the following steps.

- Start the relex server, using for instance docker

   ```
   <DOCKER_REPO>/opencog/relex/run-opencog-relex.sh
   ```

- Either start the cogserver and enter the scm shell, or start guile directly

- Load the nlp module

    ```
    (use-modules (opencog nlp))
    ```

- In the opencog scheme shell run (relex-parse "some sentence"),
  preferably sentences on which the rules are applicable.

- Run (cog-bind rule-which-should-be-a-BindLink)
