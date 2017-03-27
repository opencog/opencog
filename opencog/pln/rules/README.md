# PLN Rule definitions for the URE

Implements PLN rules according to
http://wiki.opencog.org/w/URE_Configuration_Format

## Contents

This folder contains PLN rules such as

- fuzzy conjunction
- deduction
- inversion
- abduction
- modus ponens
- contextual
etc

the `wip` subfolder contains rules that are to be completed or removed.

## PLN Example

- Load the deduction rule definitions:

    ```
    opencog/pln/rules/deduction.scm
    ```

    Example:

    ```
    (load "opencog/pln/rules/deduction.scm")
    ```

- Load this file containing the data:

    ```
    tests/pln/rules/simple-assertions.scm
    ```

    Example:

    ```
    (load "tests/pln/rules/simple-assertions.scm")
    ```

- Run this command:

    ```
    (cog-bind find-humans)
    ```

- Observe that there is only one instance of human defined:

    ```
    (ConceptNode "man")
    ```

- Run this command:

    ```
    (cog-bind deduction-rule)
    ```

- Run this command again:

    ```
    (cog-bind find-humans)
    ```

- Observe that there are 3 additional instances of human defined:

    ```
    (ConceptNode "Socrates")

    (ConceptNode "Einstein")

    (ConceptNode "Peirce")
    ```

There are more examples under the folder

   ```
   examples/pln
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
