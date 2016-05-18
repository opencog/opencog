Running AIML in OpenCog
-----------------------

This directory includes assorted infrastructure for running AIML-style
rules within OpenCog.  It assumes that AIML data has already been
imported, using the tools in `import` directory.

Status
------
Ongoing hackery


Running
-------



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
The OpenPsi-based rule importer represents a typical AIML rule in the
following form:
```
(Implication
	; context
   (List (Concept "I") (Glob "$star") (Concept "you"))
	; goal
   (List (Concept "I") (Glob "$star") (Concept "you") (Concept "too"))
	; demand
	(Concept "AIML chatbot demand")
)
```

To find this rule, we need to search for it.  This can be accomplished
by saying:
```
(cog-execute!
	(Dual
		(List (Concept "I") (Concept "love") (Concept "you"))))
```
which will find the above rule (and any other rules) that match this
pattern. Alternately, there is a convenience wrapper:
```
(psi-get-dual-rules
	(List (Concept "I") (Concept "love") (Concept "you"))))
```

But first, before we can do this, we must find the current
sentence and parse in the atomspace.  This can be done by saying:
```
(Get
   (VariableList
      (TypedVariable (Variable "$sent") (VariableType "SentenceNode"))
      (TypedVariable (Variable "$parse") (VariableType "ParseNode"))
      (TypedVariable (Variable "$tok-seq") (VariableType "ListLink"))
   )
   (And
      (State (AnchorNode "*-eva-current-sent-*") (Variable "$sent"))
      (ParseLink (Variable "$parse") (Variable "$sent"))
      (Evaluation
         (PredicateNode "Token Sequence")
         (Variable "$parse")
         (Variable "$tok-seq"))
   ))
```

Running the above will return the sequence of words that have
been recently uttered.

## Rule application
Having found all of the rules, we now have to run them ... specifically,
we have to apply them to the current parse.  We can do this in one of
several ways.  These are:

* Create a NEW bindlink, which mashes together both the search for the
  current parse, and the AIML rule, and generate the resulting output,
  which then needs to be routed to the chatbot, to be voiced.

* Since we already know the current parse, we can limit the application
  of the naive AIML rule to only that parse.  This requires treating the
  rule as a kind of PutLink, i.e. of applying it only to a specified
  set.

Misc Notes
----------
Hints for hand-testing this code:

```
(load "run.scm")
(use-modules (opencog) (opencog nlp) (opencog openpsi))
(load "/tmp/aiml.scm")
```
Except that loading the aiml rules triggers a compile, which takes too
long.  So need to disable compiling.


Search for the rules:
```
;; YOU CAN DO BETTER
(psi-get-dual-rules (List (Word "you") (Word "can") (Word "do") (Word "better")))
```
write a quickie tokenizer:
```
(map
	(lambda (w) (Word w))
	(string-split "you can do better" (lambda (x) (eq? x #\ ))))

(define (tokenize SENT-STR)
	(ListLink (map
		(lambda (w) (Word w))
		(string-split SENT-STR (lambda (x) (eq? x #\ ))))
))

(tokenize "you can do better")

```

TODO:
* disable compiling when loading aiml rules (or compile in background)
