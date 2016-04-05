Running AIML in OpenCog
-----------------------

This directory includes assorted infrastructure for running AIML-style
rules within OpenCog.  It assumes that AIML data has already been
imported, using the tools in aiml2oc directory.

Status
------
In development, experimental, pre-alpha.


Design Notes
------------

## Input
Sentences from RelEx.  That is, the normal RelEx and R2L pipeline
runs, creating a large number of assorted atoms in the atomspace
representing that sentence.  See the RelEx and R2L wiki pages for
documentation.

## Pre-processing
The script `make-token-sequence` creates a sequence of word tokens from
a given RelEx parse.  For example, the sentence "I love you" can be
tokenized as:
```
(Evaluation
   (PredicateNode "Token Sequence")
   (Parse "sentence@3e975d3a-588c-400e-a884-e36d5181bb73_parse_0")
   (List
      (Concept "I")
      (Concept "love")
      (Concept "you")
   ))
```

The above form is convenient for the AIML rule processing, as currently
designed.  Unfortunately, it erases information about specific words,
such as part-of-speech tags, tense tags, noun-number tags, etc.  Such
information could be available if we used this form instead:

```
(Evaluation
   (PredicateNode "Word Sequence")
   (Parse "sentence@3e975d3a-588c-400e-a884-e36d5181bb73_parse_0")
   (List
      (WordInstance "I@a6d7ba0a-58be-4191-9cd5-941a3d9150aa")
      (WordInstance "love@7b9d618e-7d53-49a6-ad6e-a44af91f0433")
      (WordInstance "you@cd136fa4-bebe-4e89-995c-3a8d083f05b6")
   ))
```

However, we will not be using this form at this time, primarily because
none of the AIML rules to be imported are asking for this kind of
information.

## Rule selection

## Rule application
