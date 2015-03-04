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
(nlp-parse "He eats.")
(nlp-parse "He eats quickly.")
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
sentences mean those that were inputed with `(nlp-parse ...)` calls.  The
`nlp-parse` call will uses LG, RelEx, and RelEx2Logic to generate the necessary
atoms.

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
(ReferenceLink
   (LgLinkInstanceNode "Ds**c@4432ef3-3c4e-42a9-8072-9975d168a12c")
   (LinkGrammarRelationshipNode "Ds**c")
)

```

are generated.  They contain all the information of the LG connectors used for
a particalar word of a particalar sentence.

See the scheme function `word-inst-get-source-conn` to see how the information
can be extracted.

Given a new `SetLink` as input, SuReal matches each atom in the link to the
structure inside the old sentences.  In addition, for each node that it
matches, it check the word corresponding to the node and see if its LG disjuncts
agree with the usage of the word it is replacing.


## Node & Link

```
(EvaluationLink (stv 1.0 1.0)
   (LgLinkInstanceNode "Ds**c@4432ef3-3c4e-42a9-8072-9975d168a12c")
   (ListLink
      (WordInstanceNode "the@da65d87c-22b9-4af2-89f4-60042816c579")
      (WordInstanceNode "man@1a6d58eb-e9c0-4c8e-af01-8d4304e3430c")
   )
)
```
The `EvaluationLink` above describes one instance of the LG link used
to link the two words in the sentence.

```
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
```
The `LgLinkInstanceLink` further explain the `EvaluationLink` by indicating
what the original LG connector is.  For example, "the" is using "D+" to
connect with "Ds**c-" of "man".

```
(ReferenceLink
   (LgLinkInstanceNode "Ds**c@4432ef3-3c4e-42a9-8072-9975d168a12c")
   (LinkGrammarRelationshipNode "Ds**c")
)
```
The `ReferenceLink` will just link the LG link instance to a general version of
the LG link.

In addition to the above links, SuReal also uses **LG-Dict** to generate the
disjuncts for each input word in the atomspace.

Note that currently there is also

```
(EvaluationLink (stv 1 0.99999982)
   (LinkGrammarRelationshipNode "Ds**c")
   (ListLink
      (WordInstanceNode "the@da65d87c-22b9-4af2-89f4-60042816c579")
      (WordInstanceNode "man@1a6d58eb-e9c0-4c8e-af01-8d4304e3430c")
   )
)
```
for language learning purpose.

