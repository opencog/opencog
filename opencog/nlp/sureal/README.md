# Surface Realization

Functions for surface realization (SuReal) of output from microplanner's output.
The microplanner also uses SuReal to determine whether a chunk (a set of atoms)
can be translated into a sentence, so the two projects are dependent on each
other.

The main function is `sureal` which takes in a `SetLink` and returns a
sentence.

The words used in the input `SetLink` need to have the corresponding `WordNode`
or `WordInstanceNode` before calling `sureal`.

For example, you can do

```
(r2l "He eats.")
(r2l "He eats quickly.")
(WordNode "she")
(WordNode "drinks")
(sureal (SetLink (EvaluationLink (PredicateNode "drinks") (ListLink (ConceptNode "she")))))
```
which will return all possible sentence is words list, like

```
((she drinks .) (she drinks quickly .))
```

## Algorithm

SuReal does pattern matching on existing sentences in the atomspace.  Existing
sentences mean those that were inputed with `(r2l ...)` calls.  The `r2l` call
will uses LG, RelEx, and RelEx2Logic to generate the necessary atoms.

Especially important is the LG outputs.  For each sentence, a bunch of links of
the style

```
(EvaluationLink (stv 1.0 1.0)
   (LgLinkInstanceNode "Ds**c@4432ef3-3c4e-42a9-8072-9975d168a12c")
   (ListLink
      (WordInstanceNode "the@da65d87c-22b9-4af2-89f4-60042816c579")
      (WordInstanceNode "man@1a6d58eb-e9c0-4c8e-af01-8d4304e3430c")
   )
)
(LgLinkInstanceLink
   (LgLinkInstanceNode "Ds**c@4432ef3-3c4e-42a9-8072-9975d168a12c")
   (LgConnector
      (LgConnectorNode "D")
      (LgConnDirNode "+")
   )
   (LgConnector
      (LgConnectorNode "Ds**c")
      (LgConnDirNode "-")
   )
)
(EvaluationLink
   (PredicateNode "LgLinkInstanceType"
   (ListLink
      (LgLinkInstanceNode "Ds**c@4432ef3-3c4e-42a9-8072-9975d168a12c")
      (LinkGrammarRelaationshipNode "Ds**c")
   )
)

```

are generated.  They contain all the information of the LG connectors used for
a particalar word of a particalar sentence.

Given a new `SetLink` as input, SuReal matches each atom in the link to the
structure inside the old sentences.  In addition, for each node that it
matches, it check the word corresponding to the node and see if its LG disjuncts
agree with the usage of the word it is replacing.

