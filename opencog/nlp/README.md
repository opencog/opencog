
Natural Language Processing
===========================

This directory contains an assortment of natural language processing
subsystems, primarily focused on the English language. This includes
language learning code, logical reasoning on facts extracted from
sentences, reference resolution, question answering, natural
language output (generation of syntactically valid sentences),
a chatbot interface, and some AIML interfaces.

How-To
------
The current code "doesn't do anything yet"; rather, it is a platform for
running experiments. Thus, what is contained here should be thought of
as a "bag of parts".  It is up to you to figure out what these parts
are, and to assemble them into something meaningful.  Some fragile
examples have been constructed (e.g. the code in the old-chatbot
directory) but its up to you to do better.

Currently, there are three basic experimental flows that are possible:
the chatbot process, the language-learning process, and the
word-sense-disambiguation (WSD) process.  See the "learn" directory for
more about language learning. See the wsd directory for more about wsd.

A general sketch
----------------
A detailed sketch of the desired dialog system can be found in the PDF
file in the "dialog_system" directory.

A general sketch of the desired system:

 * Every sentence has one or more parses. (e.g. "I saw the man with the
   telescope") -> "the man was carrying a telescope" or "I used to
   telescope to look".  This works, multiple parses are returned by
   relex2logic.

 * A given parse may have multiple, different "interpretations"  (e.g.
   "I saw" -> "I  use a saw-blade to cut..." or "I use my eyes to
   look-at...")

 * Rank interpretations according to likelihood.  The simplest algo is
   the google page-rank algo ("Mihalcea") which simply says
   "saw-looking and telescope are similar but saw-cutting and
   telescope are not similar, therefor saw-looking is much more likely"
   Code for this is in nlp/wsd directory, but it is *very very old* and
   not compatible with current infrastructure.

 * Anaphora resolution.  "My name is Romain. "I saw..." deduce "I" is
  "Romain".  Code in nlp/anaphora directory. Should kind-of-ish work,
   hasn't been tested for years. Its not hooked up.

 * Reference resolution "saw the man" ->  "the man" is actually a
   creeper in the minecraft world. There's some abandoned code for
   minecraft; its not working right now.  The Hanson Robotics
   "chatbot-eva" has a some hand-coded stuff that can see people via
   web-cam, and listen via microphone.  Under active development.

 * PLN does not only logic, but helps clarify interpretations and
   reference resolution, and disambiguates word-meanings.  So its a
   mess, for now.  Its not just "hooking up PLN", its hooking it up
   so that these other parts might eventually work right.

 * High-level dialog structure: the rheme/theme structure, e.g.
   "I am saying X about Y", and more generally dialectics (how will
   I get my point across).

 * Learning. The need to associate verbal acts with specific
   physical-world actions, and physical world objects.


Subdirectories (in alphabetical order)
======================================
aiml
----
Mothballed code for converting AIML XML into opencog "atomese", so that
a simple opencog script could run AIML dialogs.  There is no compelling
AIML content that is in any way useful to the current plans for OpenCog.
I think that we've moved past AIML in terms of what we can accomplish.

anaphora
--------
Code that picks out pronouns (he, she, it) and offers up a list of
plausible candidates that these might refer to.

chatbot
-------
Scripts for running the current IRC or Telegram-based chatbot.

chatbot-eva
-----------
Scripts for running a mashup of the IRC chatbot, attached to the
Hanson Robotics Sophia head.  Can obey a very small set of direct
commands, e.g. "Smile! Act sad! Look afraid! Look left!".

chatbot-old
-----------
The chatbot-old directory implemented a question-answering system.
It could answer factual questions, e.g. "Tom threw a ball. Who threw a
ball?" It could answer a variety of questions, and came with a small
common-sense database.  It implemented a full processing pipeline:
IRC chatbot -> link-grammar parse -> grammatical pattern matcher.
As long as a question fit the grammatical pattern of a known answer,
it could answer that question.

dialog_system
-------------
A PDF discussing the structure of the desired dialog system.

diary
-----
Research notes.

fuzzy
-----
A system for performing some "fuzzy" grammatical pattern matching.
The idea was that if there was a question, and grammatical pattern
machining cannot find an exact answer in the AtomSpace, then perhaps
a "fuzzy" match might find something "good enough". Underwhelming.

irc
---
Interfaces to IRC; provide an easy way to connect opencog to IRC.

lojban
------
A Lojban parser.

microplanning
-------------
A planner for generating natural language sentences from a bag of
dependency-grammar relations. That is, given a set of dependencies
that express an idea, the planner will create a series of sentences that
express the idea, are syntactically correct, are of appropriate length,
and use pronouns in an appropriate way.

relex2logic
-----------
Convert RelEx dependency relations to logical predicates/assertions. The
point here is that PLN can only reason on logical forms, and so we need
to convert dependencies into (first-order, probabilistic) logic forms.
Relex2Logic consists of a bunch of hand-authored rules, but these are a
bit problematic. The system is semi-deprecated, but does not yet have a
viable alternative.

scm
---
The scm directory contains miscellaneous scheme scripts of general
utility for NLP work.

sentiment
---------
Some scaffolding for integrating 3rd party sentiment analysis tools.

sureal
------
Surface realization -- the lowest level portion of natural language
generation. Works in conjunction with the microplanner to build
sentences.

types
-----
Scripts specifying NLP-specific atom types.

wsd
---
The wsd directory contains code that implements the Rada Mihalcea
word-sense disambiguation algorithm.  The Mihalcea algorithm assigns
senses to the words in a sentence, and constructs links between these
different senses. The links are weighted by a word-sense similarity
measure. The result is a graph whose vertices are word-senses, and
whose edges are these links.  The graph is essentially just a Markov
chain, and is solved as such, looking for a stationary vector of
probabilities for the word-sense vertices. The vertices with the
highest scores are then the most likely meaning for a word.

wsd-post
--------
The wsd-post directory contains code for generating datasets which may
be used for extremely fast (but partial) word-sense disambiguation,
based on grammatical (syntactical) usage. Generating the data-sets can
take cpu-month to cpu-years; the table-lookup can be done in milli or
microseconds.


A Side-note About Syntactic Sugar
=================================
This has been said before, but it bears repeating. Consider the node
type `WordInstanceNode`, for example:
```
    (WordInstanceNode "cabin@99d22336-6cda-4365-8555-64260ed8bd15")
```
This custom-defined node type should be thought of as syntactic sugar
for the more "primitive" graph:
```
    (InheritenceLink
        (ConceptNode "cabin@99d22336-6cda-4365-8555-64260ed8bd15")
        (ConceptNode "WordInstance")
    )
```
The above InheritenceLink essentially assigns a "type" to the word
instance. This type can be used in the same way that types are
ordinarily used in other programming languages. When managing
hypergraphs, it is almost always easier and faster to locate,
manipulate and delete narrowly typed atoms.

Similarly, for links, we use the syntactic sugar

    (PartOfSpeechLink
        (WordInstanceNode "cabin@99d22336-6cda-4365-8555-64260ed8bd15")
        (DefinedLinguisticConceptNode "noun")
    )

which stands for the more "primitive" construct:

    (EvaluationLink
        (PredicateNode "PartOfSpeech"
            (ListLink
                (WordInstanceNode "cabin@99d22336-6cda-4365-8555-64260ed8bd15")
                (DefinedLinguisticConceptNode "noun")
            )
        )
    )

Abstractly, these forms should be considered to be "equivalent",
although there is a bunch of actual code that depends on the one or
the other, and there is no automated conversion between these forms.

References:
===========
* "To verbize one's nouns" -- the concept of "Lexical Implication Rules":
N. Ostler, B.T.S.Atkins, "Predictable Meaning Shift: Some Linguistic
Properties of Lexical Implication Rules", (1991) Proceedings of the
First SIGLEX Workshop on Lexical Semantics and Knowledge Representation
