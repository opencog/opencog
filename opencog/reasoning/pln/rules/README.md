# Prototype rule definitions for rule engine based PLN implementation

Implements rules in the AtomSpace using the pattern matcher for
unification and GroundedSchemaNodes for execution of formulas written
in Scheme.

## Rules

The following rules are defined for PLN:

    - pln-rule-and
    - pln-rule-not
    - pln-rule-deduction
    - pln-rule-modus-ponens
    - Context rules:
        - pln-rule-contextualize-inheritance
        - pln-rule-contextualize-evaluation
        - pln-rule-contextualize-subset
        - pln-rule-decontextualize-inheritance
        - pln-rule-decontextualize-evaluation
        - pln-rule-decontextualize-subset
        - pln-rule-context-free-to-sensitive
        (here, the exact formula is still unclear)

## Additional instructions

After loading the rules, you should also load this Scheme file:

```
compile-rules.scm
```

in order to compile the rules and formulas for better performance.

## PLN Example

- Load the deduction rule definitions:

    ```
    reasoning/pln/rules/deduction.scm
    ```

    Example:

    ```
    (load "reasoning/pln/rules/deduction.scm")
    ```

- Load this file containing the data:

    ```
    tests/reasoning/engine/simple-assertions.scm
    ```

    Example:

    ```
    (load "tests/reasoning/engine/simple-assertions.scm")
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
    (cog-bind pln-rule-deduction)
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

## Relex2Logic Example

The r2l subdirectory has the initial specification of the
R2L-RuleBase for the English language.

To test the rules, use the following steps.

- Start the cogserver

- Enter the scm shell.

- Load the scheme files in this directory and the sub-directories using

    ```
    (load "../path/to/scheme/files")
    ```

- Start the relex server using the --relex flag (don't use the --logic flag)

- In the opencog scheme shell run (relex-parse "some sentence"),
  preferably sentences on which the rules are applicable.

- Run (cog-bind name-of-the-variable-which-has-BindLink-as-its-value)
