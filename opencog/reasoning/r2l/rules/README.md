# Prototype rule definitions for rule engine based R2L implementation

Implements rules in the AtomSpace using the pattern matcher for
unification and GroundedSchemaNodes for execution of formulas written
in Scheme.

## Rules

The following rules are defined for Relex2Logic:

- Relex2Logic:
    - be-inheritance
    - svo

## Additional instructions

After loading the rules, you should also load this Scheme file:

```
compile-rules.scm
```

in order to compile the rules and formulas for better performance.

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
