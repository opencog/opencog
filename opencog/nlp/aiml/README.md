
# AIML in the AtomSpace

There are recurring, assorted requests to be able to script chatbot
interactions using AIML, and this directory contains the tools to
support such capability.

Please observe that using AIML is very much AGAINST the spirit of
OpenCog, and so these tools are only grudgingly provided.  The real
goal of OpenCog is to create a machine that can learn and think, rather
than run off of hard-coded, human-created rules.  Of course, we've
de-facto violeted this rule repeatedly: the link-grammar rules are
human-created and hard-coded, as are the relex and relex2logic rules.
Alas. So perhaps using AIML is no real crime.

## Status
Beta.  Most things work. There are known bugs. See TODO list below.
It attempts to support the most commonly used features of AIML version 2.1

## Using
There are two steps to this process: The import of standard AIML markup
into the AtomSpace, and the application of the imported rules to the
current speech (text-processing) context.

The goal of the import step is to map AIML expressions to equivalent
atomspace graphs, in such a way that the OpenCog pattern matcher can
perform most or all of the same functions that most AIML interpretors
perform. The goal is NOT to re-invent an AIML interpreter in OpenCog!
... although, de-facto, that's what the end-result is: an AIML
interpreter running in the AtomSpace.

The current importer generates OpenPsi rules, so that the selection
and execution of the AIML rules is performed by OpenPsi (see the
[OpenPsi directory](../../openpsi) for details.) This allows
the AIML chat subsystem to be inter-operated together with other
subsystems (for example, the on-verbal robot behavviors).

## Running AIML in OpenCog
AIML files need to be converted into Atomese, using the perl script
provided in the [`import`](import) directory.  This  can be done as:
```
import/aiml2psi.pl --dir /where/the/aiml/files/are
cp aiml-rules.scm /tmp
```
Then do this:

```
(use-modules (opencog) (opencog nlp) (opencog nlp aiml) (opencog openpsi))
(primitive-load "/tmp/aiml-rules.scm")
```
For basic debugging, the `string-words` utility can be used to tokenize
a string into words; its very low-brow and basic:
```
(aiml-get-response-wl (string-words "call me ishmael"))
```
Proper integration with the normal language pipeline requires the use
of the `token-seq-of-parse` and `token-seq-of-sent` functions, which
get the word-sequence as determined by the link-grammar parser (i.e.
obtain the word sequence from the normal OpennCog NLP ppipeline.)

The various sections below provide additional details about how the
system is implemented and how it works.


## Lightning reivew of AIML
Some example sentences.

```
        Input               That Topic     Result
(R1)  Hello                  *     *    Hi there
(R2)  What are you           *     *    I am Hanson Bot
(R3)  What are you thinking  *     *    How much I like you
(R4)  Hi                     *     *    <srai>Hello</srai>
(R5)  I * you                *     *    I <star/> you too.
(R6)  What time is it        *     *    The time is <syscall>
(R7)  *                      *     *    I don't get it.
(R8)  ...                    *     *    <set topic = "animals dog"/>
(R9)  ...                    *     *    <set name="topic">animals dog</set>
(R10) ...                    *     *    <set name="topic"><star/></set>
(R11) ...                    *   "* dog"   ...
(R12) ...                    *     *    I like <get name="topic"/>
(R13) ...                    *     *    I like <bot name="some_key"/>
(R14) Is it a <set>setname</set> ?
(R15) Are you <bot name="key"/> ?
(R16) Are you <set><bot name="species"/></set> ?
```

Future, not a current part of AIML:
```
(F1)  Do you like <topic/> ?
(F2)  Do you like <get name="topic"/> ?
```

### Notes
* Notice that the pattern match of R3 takes precendence over R2.
* srai == send the result back through. ("stimulus-response AI")
* that == what the robot said last time. (just a string)
  A full history of the dialog is kept, it can be refered to ...
  (TBD XXX How ???)
* topic == setable in the result. (just a string)
  topic is just one key in a general key-value store of per-conversation
  key-value pairs.

* star and underscore (`*` `_`) are essentially the same thing; it's an
  AIML implementation detail.
* star means "match one or more (space-separated) words"
  stars are greedy.
* carat and hash (`^` `#`) mean "match zero or more words"
* There is an implicit star at the end of all sentences; it is less
  greedy).
* (R11) the wild-card match matches the second of two words in a
  2-word topic.  (TBD XXX what about N words ??)
* (R14) the wild-card match limited to one of a set.


## Pattern Recognition

Implementing AIML efficiently in the atomspace requires that the pattern
matcher be run "in reverse": rather than applying one query to a
database of thousands of facts, to find a small handful of facts that
match the query; we instead apply thousands of queries to a single
sentence, looking for the handful of queries that match the sentence.
In this sense, AIML is an example of pattern recognition, rather than
querying.  The traditional pattern-matcher link types `BindLink`,
`GetLink` and `SatisfactionLink` are not appropriate; instead, the
underlying search is performed with `DualLink`.

Given a graph (containing only constants -- that is, a "ground term")
the `DualLink` mechanism will find all graphs with variables in them
such that these graphs might be grounded by the given graph.
See `http://wiki.opencog.org/w/DualLink` for details.


### Globbing

The pattern matcher uses GlobNode to perform globbing.
See `http://wiki.opencog.org/w/GlobNode` for details.
See `glob.scm` for a simple working example.


## OpenPsi general format
OpenPsi rules have the following general format:
```
ImplicationLink
   AndLink
      context
      action
   goal
```
The context must be fullfilled (at least approximately) in order for
the action to be taken.

## OpenCog AIML equivalent
* R5 example.

The R5 sentence in the example above is converted into the following
OpenPsi fragment:

```
ImplicationLink
   AndLink
      SetLink
         ListLink
            Word "I"
            Glob "$star"
            Word "you"
      ListLink
         Word "I"
         Glob "$star"
         Word "you"
         Word "too"
   Concept "AIML chat subsystem goal"
```

The SetLink contains all of the context that must be fulfilled, in order
for a rule to be chosen.  In this case, the Set contains only one item:
the list of words in the sentence.  For AIML rules that have a
non-trivial topic, the Set contains atoms to match the topic. For AIML
rules that have a non-trivial "that" section, the Set would also contain
atoms that match previous the sentence.


### AIML tags
All AIML tags are converted into `DefinedSchema` nodes.  There are
processing routines to carry out the actions.  For example, the
&lt;person&gt; tag gets converted to
```
      (ExecutionOutput
         (DefineSchema "AIML-tag person")
         (ListLink
             (Glob "$star-1")))

```


##Design Notes

### Input
Sentences from RelEx.  That is, the normal RelEx and R2L pipeline
runs, creating a large number of assorted atoms in the atomspace
representing that sentence.  See the RelEx and R2L wiki pages for
documentation.

### Pre-processing
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

### Rule selection
The OpenPsi-based rule importer represents a typical AIML rule in the
following form:
```
(Implication
	(AndLink
		; context
		(SetLink
			(List (Concept "I") (Glob "$star") (Concept "you"))
		)
		; action
		(List (Concept "I") (Glob "$star") (Concept "you") (Concept "too"))
	)
	; goal
	(Concept "AIML chatbot goal")
)
```
The actual representation is a bit more complicated; the above captures
the general idea of a psi-rule, as needed for AIML processing.

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
(psi-get-members
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

### Rule application
Having found all of the rules, we now have to run them ... specifically,
we have to apply them to the current parse.  We can do this in one of
several ways.  One is to use a BindLink, the other is to use a MapLink.
The MapLink is sort-of like a BindLink, but it is applied only to its
argument set, instead of being applied to the entire atomspace.

* The BindLink approach requires that the current parse be tagged in the
  atomspace, and a BindLink created that mashes together the current
  tag, and the rule to be applied.  The response(s) are then the result
  of evaluating the BindLinks.

* The MapLink approach is similar, except that the rule becomes the body
  of the apLink, and the current sentence is given as the argument to
  the MapLink.

Either approach should work fine. Currently, the MapLink approach is
being used, for no particular reason other than to see how it compares
to a BindLink approach for ease-of-use.


### TODO
* OpenPsi DualLink is returning too much -- i.e. it includes matches to
  the action.  We really only want matches to the context only.
  An extended version of the `get-pattern-rules` tool could do this
  kind of filtering.  Review with Amen, maybe open bug report.
* OpenPsi rule design is kind-of broken -- in two ways:
  (1) the action should be immediately obtainable from the rule,
  instead of fishing around for it via the psi-action? utility.
  (2) If an action is a schema, it should not appear in an AndLink,
  because its not evaluatable.
  (3) Actions are naturally ordered sequences, and that means
  SequentialAnd and that meands a predicate, not a schema.
* AIML -- implicit * at end of sentence, can be NULL.
  (I guess globbing should work with that...?)
* AIML -- thatstar and topicstar not handled.
* OpenPsi -- need a general mechanism that avoids doing the same thing
  over and over (e.g. saying the same thing). Discusss with Ben, Amen.
  After this is done, remove the duplicate-sentence filter from the
  get-repsonses method 9which also sets "that"
* general utility -- create an is-member? utility to replace psi-action?
* integration: any AIML xfind response should be handled by
  the fuzzy matcher instead...
* AIML: should use fuzzy matching when there are no globs.
* Invent a fuzzy matcher that does support globs.

### BUGS
* "agians" takes too long -- psi-get-member-links is taking too long
  when there are many rules. -- get-iset taking too long.
  ("agians" is a rule in the futurist set of rules).

### Done
Completed tasks, but double check!?
* AIML HR -- in a session, never say the same thing twice!
  Done -- for just one sentence .. ask Vytas about more.
* AIML HR -- load only the current, desired rule-set. Get someone
  to write shell scripts or config files for this (Wenwei?)
  Per config files in `chathub/server/characters/sophia.properties`
  (It seems that all of the files are currently loaded).

### HR Demo
```
import/aiml2psi.pl --dir ./chathub/generic_aiml/ --outfile generic.scm --priority=0.9
import/aiml2psi.pl --dir ./chathub/futurist_aiml/ --outfile futurist.scm --priority=0.6
guile
(use-modules (opencog) (opencog nlp) (opencog nlp aiml) (opencog openpsi))
(primitive-load "/tmp/generic.scm")
(primitive-load "/tmp/futurist.scm")
```


### Misc Notes

Hints for hand-testing this code:

Pre-compile the AIML files (this can take over an hour)
(This is optional)
```
time guild compile "/tmp/aiml.scm"
```

```
(use-modules (opencog) (opencog nlp aiml))
```
A normal file load triggers a compile; avoid this as follows:
```
(primitive-load "/tmp/aiml.scm")
(use-modules (system base language))
(compile "/tmp/aiml.scm")
(load-compiled (compiled-file-name "/tmp/aiml.scm"))

```

Search for the rules:
```
;; YOU CAN DO BETTER
(psi-get-dual-rules (List (Word "you") (Word "can") (Word "do") (Word "better")))
(string-words "you can do better")
(string-words "you are such a winner")
```
Search for duals by hand:
```
(define sent
	(List (Word "you") (Word "can") (Word "do") (Word "better")))

(cog-execute! (DualLink sent))

(cog-incoming-set sent)
(cog-get-root sent)

(define s3 (string-words "who supports Trump?"))
(define s3a (string-words "who endorses Trump?"))
(define s4 (string-words "who won the superbowl"))
(aiml-get-response-wl s4)
```


###Notes:

```
psi-get-dual-rules calls psi-get-member-links

(cog-execute! (car (aiml-get-response-wl s3)))

(use-modules (opencog) (opencog nlp) (opencog exec))
(cog-execute! (Map
	(Implication
		(ListLink (Word "who") (Word "supports") (Glob "$star-1"))
		(ListLink (Word "who") (Word "endorses") (Glob "$star-1")))
	(Set (ListLink (Word "who") (Word "supports") (Word "Trump")))
))

(cog-execute! (Map
	(Implication
		(ListLink (Word "who") (Word "supports") (Glob "$star-1"))
		(ExecutionOutput (DefinedSchema "AIML-tag srai")
			(ListLink
				(ListLink (Word "who") (Word "endorses") (Glob "$star-1")))))
	(Set (ListLink (Word "who") (Word "endorses") (Word "Trump")))
))

(aiml-get-response-wl
	(ListLink (Word "who") (Word "endorses") (Word "Trump")))

(psi-rule
   ;; context
   (list
      (ListLink
         (Word "who")
         (Word "endorses")
         (Glob "$star-1")
      ) ; PATEND
   ) ;TEMPATOMIC
   ;; action
   (ListLink
      (Word "The")
      (Word "crazies")
      (Word "like")
      (Glob "$star-1")
   ) ; TEMPATOMICEND
   (Concept "AIML chat goal")
   (stv 1 0.8)
   (psi-demand "AIML chat" 0.97)
) ; CATEND

(aiml-get-response-wl (string-words "will you remember what"))
(aiml-get-response-wl (string-words "what will you remember"))
(aiml-get-response-wl (string-words "will you remember that"))
MAY I TEACH YOU
REMEMBER THAT

(aiml-get-response-wl (string-words "you do not learn"))
(aiml-get-response-wl (string-words "call me ishmael"))

-- non-trivial that:
THAT IS A GOOD PARTY

(use-modules (ice-9 ftw))

(define (load-all-files DIR)
	(map
		(lambda (fn) (primitive-load (string-join (list DIR fn) "/")))
		(scandir DIR (lambda (fname) (string-contains fname ".scm"))))
	#t)

(load-all-files "/home/linas/src/opencog/opencog/nlp/aiml/import/aiml-scm/")
```

==================
WHEN WILL YOU * BODY

(aiml-get-response-wl (string-words "call me ishmael"))
(aiml-get-response-wl (string-words "who supports Trump?"))

DO YOU KNOW BOTNAME

thatstar/>

topicstar/>

114690a114709 -- condition-response

<pattern>_</pattern> <topic>BADANSWER</topic>
===================================
; Context with topic!
; Context with that!

(do-aiml-get (Concept "that"))
(do-aiml-get (Concept "topic"))
(do-aiml-set (Concept "topic")   (string-words "ends with alice"))
(aiml-get-response-wl (string-words "test botname"))

(do-aiml-get (Concept "name"))


;;; <category><pattern>TEST BOTNAME</pattern> <topic>ENDS WITH
ALICE</topic> <that>*</that> <template><bot name=\"name\"/> is
functioning normally. <think><set name=\"matched\">true</set></think>
</template> </category>

(do-aiml-bot-get (Concept "name"))
(do-aiml-get (Concept "name"))
(aiml-get-response-wl (string-words "my name is joob"))
(aiml-get-response-wl (string-words "call me joob"))

=================================

OpenPsi predicate/schema issues!

(Define (DefinedPredicate "pblorf")
(SequentialAnd
(Evaluation (GroundedPredicate "scm: print-msg-face")
         (ListLink (Node "--boaty")))`
(Evaluation (GroundedPredicate "scm: print-msg")
         (ListLink (Node "--mcboat")))
(True)
))

(cog-execute! (DefinedPredicate "pblorf"))

=================================
(aiml-get-response-wl (string-words "what are you doing tonight?"))
(define SENT (string-words "what are you doing tonight?"))

(use-modules (opencog) (opencog exec) (opencog nlp))

(aiml-get-response-wl (string-words "what was that sound?"))
(aiml-get-response-wl (string-words "what is a bludgeon?"))
(aiml-get-response-wl (string-words "what do you think?"))
(aiml-get-response-wl (string-words "where is the ball?"))
(aiml-get-response-wl (string-words "when will you know?"))
(aiml-get-response-wl (string-words "How is it going?"))

test topic:
set topic:
(aiml-get-response-wl (string-words "when will you astronaut body"))
(do-aiml-get (Concept "topic"))
(do-aiml-set (Concept "topic") (ListLink (WordNode "astronaut")))

(aiml-get-response-wl (string-words "i am an astronaut"))
(define SENT (string-words "i am an astronaut"))
(psi-get-exact-match SENT)
