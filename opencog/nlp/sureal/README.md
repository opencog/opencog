# Surface Realization

Functions for surface realization (SuReal) of output from the
microplanner's output.  The microplanner also uses SuReal to determine
whether a chunk (a set of atoms) can be translated into a sentence;
thus, these two projects are dependent on each other.

Sureal requires Link Grammar (LG), RelEx, and RelEx2Logic outputs.

The main functions are `sureal` and `cached-sureal`. The former takes
in a `SetLink` and returns a sentence while the latter takes in a
SetLink and return whether it succeeded building a proper sentence or
not (but the sentence itself is not returned). The point of
`cached-sureal` is that it is optimized to be used inside the
Microplanner while `sureal` is supposed to be used by other
general-purpose applications.

The words used in the input `SetLink` need to have the corresponding
`WordNode` before calling `sureal`.

For example, you can do

```
(nlp-parse "He eats.")
(nlp-parse "She eats quickly.")
(nlp-parse "Nobody drank it.")
(nlp-parse "It drinks water.")

(sureal (SetLink (EvaluationLink (PredicateNode "drink") (ListLink (ConceptNode "she")))))
```

and it will return

```
(("she" "drinks" "."))
```

which is the best match of the above.

Also you can specify the tense of the verb by doing

```
(sureal (SetLink (EvaluationLink (PredicateNode "drink") (ListLink (ConceptNode "she")))
    (InheritanceLink (PredicateNode "drink") (DefinedLinguisticConceptNode "past"))))
```

which will return

```
(("she" "drank" "it" "."))
```


**Note:**
Before generating sentences using SuReal, we have to make sure that
the words we are expecting exist in the AtomSpace (and are in correct
grammatical forms). For example, if we want to generate "she drinks"
as in the above example, we have to make sure the words "she" and
"drinks" are there in advance.  Similarily, in the second example,
we have to make sure "she" and "drank" exist.  Otherwise, SuReal will
return nothing. The recommended way to "add the words" into the
AtomSpace is to parse sentences that actually contain those words.
In this case, they are "She eats quickly.", "It drinks water." and
"Nobody drank it." respectively.


## Algorithm

SuReal does pattern matching on existing sentences in the atomspace.
Existing sentences mean those that were inputed with `(nlp-parse ...)`
calls.  The `nlp-parse` call will uses LG, RelEx, and RelEx2Logic to
generate the necessary atoms.

Especially important is the LG outputs.  For each sentence, a bunch of
links of the style:

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
      (LgConnNode "D")
      (LgConnDirNode "+")
   )
   (LgConnector
      (LgConnNode "Ds**c")
      (LgConnDirNode "-")
   )
)
(ReferenceLink
   (LgLinkInstanceNode "Ds**c@4432ef3-3c4e-42a9-8072-9975d168a12c")
   (LinkGrammarRelationshipNode "Ds**c")
)

```

are generated.  They contain all the information the LG connectors
used for a particular word of a particular sentence.

Given a new `SetLink` as input, SuReal matches each atom in the link to
the structure inside the old sentences.  In addition, for each node that
it matches, it check the word corresponding to the node and see if its
LG disjuncts agree with the usage of the word it is replacing.


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
      (LgConnNode "D")
      (LgConnDirNode "+")
   )
   (LgConnector
      (LgConnNode "Ds**c")
      (LgConnDirNode "-")
   )
)
```
The `LgLinkInstanceLink` further explains the `EvaluationLink` by
indicating what the original LG connector is.  For example, "the" is
using "D+" to connect with "Ds**c-" of "man".

```
(ReferenceLink
   (LgLinkInstanceNode "Ds**c@4432ef3-3c4e-42a9-8072-9975d168a12c")
   (LinkGrammarRelationshipNode "Ds**c")
)
```
The `ReferenceLink` will just link the LG link instance to a general
version of the LG link.

In addition to the above links, SuReal also uses **LG-Dict** to generate
the disjuncts for each input word in the atomspace.

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
for language learning purposes.

## TODO - Architecture and Design issues
There are several architectural problems that need to be dealt
with.  Thse are:

* The use of `SetLink` to pass around data is a design flaw.
  It should be avoided, and if unavoidable, should be replaced
  by MemberLinks.

* The question of morphology is completely ignored.  This can be
  non-trivial, since, in IndoEuropean languages, all meaning
  (semantics) is carried by the root, and all tense, number, mood
  agreement is handled by suffixes. Ignoring morphology means that
  there would be a combinatorial explosion in the size of the
  dictionaries.

  In English, morphology can be ignored, and ignoring it only
  triples the size of the dictionary: singular and plural versions
  for nouns, and -s, -ed, -ing versions of verbs. This is a small
  price to pay, for English, but becomes unworkable in French,
  where the dictionary would be maybe 20x bigger, or Russion,
  which might be 50x or 100x bigger.
