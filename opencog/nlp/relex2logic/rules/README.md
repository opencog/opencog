## Relex2Logic RuleBase
This directory has the initial specification of the R2L-RuleBase for the
English language.

To use the rules, use the following steps.

1. Start the cogserver
2. Enter the scm shell.
3. Load the rules using `(load-r2l-rulebase)`
4. Start the relex server using the --relex flag (don't use the --logic flag)
5. In the opencog scheme shell run `(nlp-parse "some sentence")`
6. Build your thing :smile:

