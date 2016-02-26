# LG-Dict

Modules and functions for generating a LG dictionary entry of a word in the atomspace.

## LGDictModule

Contains the following scheme primitives coded in C++

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

## utilities.scm

Contains more simple scheme codes for working with disjuncts.

