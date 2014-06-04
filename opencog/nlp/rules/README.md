This directory has the initial specification of the R2L-RuleBase for the english 
language.

For testing the rules, follow the follwing steps.

1. Start the cogserver
2. Enter the scm shell.
3. Load the scheme files in this directory and the sub-dirctories using
   (load-scm-from-file "../path/to/scheme/files")
4. Start the relex server using the --relex flag (don't use the --logic flag)
5. In the opencog scheme shell run (relex-parse "some sentence"), preferablly 
   sentences on which the rules are applicable.
6. Run (cog-bind name-of-the-variable-which-has-BindLink-as-its-value)
