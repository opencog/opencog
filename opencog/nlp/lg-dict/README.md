# LG-Dict

Modules and functions for generating a LG dictionary entry of a word
in the atomspace.

## Scheme module `(opencog nlp lg-dict)`

Contains the following scheme primitives:

- `(lg-get-dict-entry (WordNode "..."))`

  Takes a WordNode as input, and output the LG disjuncts in the atomspace.  The
  output is of the form:

  ```
  (SetLink
   (LgDisjunct
      (WordNode "...")
      (LgConnector
         (LgConnectorNode "Sp")
         (LgConnDirNode "-")
      )
   )
   (LgDisjunct
      (WordNode "...")
      (LgAnd
         (LgConnector
            (LgConnectorNode "Sp")
            (LgConnDirNode "-")
         )
         (LgConnector
            (LgConnectorNode "dWV")
            (LgConnDirNode "-")
         )
         ...
      )
   )
   ...
  )
  ```

  The disjuncts are in Disjuntive Normal Form (DNF)
  as described http://en.wikipedia.org/wiki/Disjunctive_normal_form
  where `LgOr` and `LgAnd` correspond to the `or` and `&` notation of LG.

  Each LG connector is fully described within the `LgConnector` link, with the
  connector name in `LgConnectorNode`, direction in `LgConnDirNode`, and the
  multi-connect property in `LgConnMultiNode`.

  The connector ordering are kept intact, as described on
  http://www.abisource.com/projects/link-grammar/dict/introduction.html
  section 1.2.1

  For more information on the Node & Link, see `nlp/types/atom_types.script`

  **Since the disjuncts are in DNF, for some words there will be an explosion
  of atoms creation (for example, up to 9000 disjuncts for a word, each
  disjunct containing 5+ connectors).**

- `(lg-conn-type-match? (LGConnector ...) (LGConnector ...))`

  Takes two `LGConnector` links as input, and check if the two connectors has
  the same type (aka. connector name).  Proper handling of subscripts &
  head/tail are included.

  The same code could have been done purely in scheme, but instead in C++ for
  performance reason (for SuReal usage).

- `(lg-conn-linkable? (LGConnector ...) (LGConnector ...))`

  Takes two `LGConnector` links as input, and check if the two connectors can
  be linked.  Two connectors can be linked only if they are of different
  direction, and their types match.

  This function does not care which connector is "-" and which is "+", as long
  as they are different.

## TODO - architecture and design issues.
The core design of this module has a number of issues, some minor, and
some pretty important.

* It currently allows only one dictionary to be open at a time, i.e.
  there can only be one single global dictionary that is open. This
  should probably be fixed. This is easily fixed by passing an
  LgDictNode as an argument.

* The `lg-get-dict-entry` returns a SetLink. It is deprecated; use
  the `lg-dict-entry` method instead. Alternately, perhaps the
  `lg-get-dict-entry` could be redesigned to return a LinkValue,
  instead?

* Many of the utilities take an explicit AtomSpace argument, and poke
  atoms into the atomspace. This is not really needed; they could more
  easily just return assorted atoms to the user, who could then perform
  a single bulk insert into whatever atomspace they need/desire. This
  would result in faster, more efficient code, since bulk inserts are
  faster than piecemeal inserts.

* The `lg-get-dict-entry` and `lg-dict-entry` methods fail to perform
  regex lookup of the word. Properly, this is a bug in the link-grammar
  API for word lookup; regexes should have been handled automatically.
  this will take a few afternoons to fix.

* The `lg-get-dict-entry` and `lg-dict-entry` do the wrong thing, or
  are invalid/inappropriate, if the word has a non-trivial mophology
  (i.e. can be split into multiple morphemes). In LG, each morpheme
  is treated as a "word", and so a single word-string can be split
  in several different ways (i.e. have multiple splittings).  Word
  splitting is non-trivial in LG.

  One way to fix this last issue is to have LG provide an API where
  the word is split into morphemes, and then the mrophemes are
  recombined, and disjuncts are returned for the recombined splits.
  That is, the morphology can be hidden "under the covers", which is
  part of the beauty of the disjunct style.

  Right now, this is low priiority, since only Russian currently has a
  non-trivial morphology, and we don't handle Russian.  The other
  reason is that sureal and microplanning are the only users of this
  system, and those are also ignorant of morphology.
