### Prototype rule definitions for a generic rule engine implementation

Implements rules in the AtomSpace using the pattern matcher for unification
and GroundedSchemaNodes for execution of formulas written in Scheme

## Rule bases

The following rule bases are currently defined:

- PLN

## Rules

The following rules are defined for the corresponding rule bases:

- PLN:
    - pln-rule-deduction
    - pln-rule-modus-ponens

## Additional instructions

After loading the rules, you should also load this Scheme file:

```
compile-rules.scm
```

in order to compile the rules and formulas for better performance.

## Next steps
- See if these can be implemented to directly use the "side-effect free" versions so that the truth value application occurs inside the ImplicationLink rather than inside the Scheme rule. This was discussed [here](https://groups.google.com/d/msg/opencog/KUptHRvBXu0/YR6oySxLKeMJ).

- See if the link type can be made to allow a dynamic list of valid link types. For example, for the Deduction Rule: {InheritanceLink, SubsetLink, ImplicationLink, ExtensionalImplicationLink}

- Support all the TruthValue types

- Utilize a graph rewriting unit test framework, that is currently being discussed, to assert that the replacement graphs match a predefined expected value for specific test instances

- For a rule like Modus Ponens to work, it will be necessary to implement "Recursive Unification using the Pattern Matcher", described [in this thread](http://wiki.opencog.org/w/Idea:_Recursive_Unification_using_the_Pattern_Matcher).

- Rules will require mutual exclusions and priorities, but that can likely be implemented outside of the definition of the rule itself for clarity.

## Example

- Load the deduction rule definitions:

    ```
    learning/RuleEngine/rules/pln/deduction.scm
    ```

    Example:

    ```
    (load-scm-from-file "learning/RuleEngine/rules/pln/deduction.scm")
    ```

- Load this file containing the data:

    ```
    tests/learning/RuleEngine/simple-assertions.scm
    ```

    Example:

    ```
    (load-scm-from-file "tests/learning/RuleEngine/simple-assertions.scm")
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
