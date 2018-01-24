# GHOST

GHOST (General Holistic Organism Scripting Tool) is a DSL (Domain-Specific Language) designed to allow human authors to script behaviors for artificial characters. GHOST is inspired by ChatScript in its syntax, but it uses OpenPsi as the engine for topic selection and topic management, i.e. action selection, as well as various other OpenCog cognitive algorithms for handling a wider range of situations in more flexible and intelligent ways.

## Design Overview

When one tries to create a rule in GHOST, it will firstly be passed to the parser (`cs-parser.scm`) for syntax checking and preliminary interpretation. Any rules that is not syntactically correct will be rejected at this stage.

The parser will then pass the intermediate interpretations (aka terms) to the translator (`translator.scm`) by calling either `create-rule` / `create-concept` / `create-topic` for creating a psi-rule / concept / topic respectively. Those terms will be converted into OpenCog atoms (defined in `terms.scm`) and stored in the AtomSpace.

An action selector is defined in `matcher.scm` for finding and selecting rules given a particular context. When a textual input is received, it will be converted into a list of WordNodes, wrapped in a `DualLink` and passed to the Recognizer for finding candidates (i.e. psi-rules) that may satisfy the current context. A full context evaluation will then be done for each of the candidates and a weight will be calculated for each of them, according to their satisfiability. Action selection will be done based on their weight and an action will be executed as a result.

## Syntax

The syntax of GHOST rules is modeled heavily on [ChatScript](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#rules). However, GHOST uses several ChatScript features for different purposes than they are normally used in ChatScript; and also contains some additional features.

Here is a list of features that are fully supported in GHOST:
- [Word/Lemma](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#canonization)
- [Phrase](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#proper-names)
- [Concept](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#concepts)
- [Choice](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#choices--)
- [Optional](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#optional-words--)
- [Indefinite Wildcard](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#simple-indefinite-wildcards-)
- [Precise Wildcard](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#precise-wildcards-n)
- [Range-restricted Wildcard](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#range-restricted-wildcards-n)
- [Variable](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#_-match-variables)
- [User Variable](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#user_variables)
- [Sentence Boundary](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#sentence-boundaries--and-)
- [Negation](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#not--and-notnot-)
- [Function](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Advanced-User-Manual.md#functions)
  - Currently only Scheme functions are accepted, can support other languages if needed
  - Should be in the public interface, e.g. use `define-public` when defining the function
  - There are several build-in functions that are available
    - `noaccess`, a topic feature that prevent any of the rules in it get triggered, unless being called explicitly. This should only be used when it's absolutely necessary.
    - `reuse`, to reuse the action of another rule, e.g.
      - `^reuse(some_label)` will reuse the action of another rule with a label named "some_label". It's recommended to use a unique label for each of the rules in the rulebase, `topic.label` is not supported.
      - Note, currently reusing a rule with local variables in the action of the rule is not supported, but user variables are fine.
    - `keep`, to keep the rule in the system so that it can be selected and executed more than once, this can be used at topic level too
- [Unordered Matching](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#unordered-matching--)


The action selection in GHOST is goal-driven, so all of the GHOST rules should be linked to one or more goals:

```
#goal: (novelty=0.8 please_user=0.4)
s: ( what be you name ) I forgot; what's YOUR name, sweet wonderful human
```

Topic level goals can also be specified:

```
goal: (please_user=0.5)
```

in which case the specified goals will be applied to every single rule under
the same topic.

Basic examples of how to use GHOST is available [HERE](https://github.com/opencog/opencog/blob/master/examples/ghost/basic.scm)


## How To Run

1) Start the [RelEx server](https://github.com/opencog/relex#opencog-serversh)
2) Start Guile
3) Load the needed modules
```
(use-modules (opencog)
             (opencog nlp relex2logic)
             (opencog openpsi)
             (opencog eva-behavior)
             (opencog ghost))
```
4) Start authoring

A rule can be created by using `ghost-parse`:

```
(ghost-parse "s: (hi robot) Hello human")
```

Similarly for creating concepts:

```
(ghost-parse "concept: ~young (child kid youngster)")
```

One can also load a topic file by using `ghost-parse-file`:

```
(ghost-parse-file "path/to/the/topic/file")
```

5) Play with it

One can quickly test if a rule can be triggered by using `test-ghost`:

```
(test-ghost "hi robot good morning")
```

The output `[INFO] [Ghost] Say: "Hello human"` will be printed.

*Note*: `test-ghost` is mainly for testing and debugging. The proper way of running it is to start the OpenPsi loop and use `ghost` instead of `test-ghost` to send the input. To do so, follow steps 1 to 4, and then do `(ghost-run)` to start the psi-loop for GHOST, and then trigger it by using `ghost`, e.g. `(ghost "how are you doing")`.
