
Generic NLP Utility Scripts
---------------------------

Assorted generic scripts related to NLP processing. These carry out
common functions, such as sending off raw text to a parser for parsing,
getting all of the words in that parse, and their parts-of-speech,
lemma forms, the RelEx relations they participate in, etc.

## Using
The code here is probvided as a scheme module.  Just say:
```
(use-modules (opencog) (opencog nlp))
```
to load this code.


## Overview

 * nlp-utils: Some mini nlp-related utilities, mostly for pointer
   chasing nlp-specific structures, e.g. getting all the parses of
   a sentence, or getting all of the word instances in a parse, and
   so on. Also:

   + delete-sentence

 * processing-utils.scm: Utilities pertaining to NLP pipeline processing.
   Defines an anchor node where newly parsed sentences can be found.
   Defines utility to send plain-text to a RelEx server to get that
   text parsed, and attaches the new parses to the anchor.

 * parse-rank.scm: Tweak link-grammar parse-ranking scores, based
   on the mutual information contained in word pairs. (Not currently
   used!)

 * type-definitions.scm: Inheritance (is-A) relations for relex types.
   Not used anywhere, currently!
