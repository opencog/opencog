# GHOST

GHOST (General Holistic Organism Scripting Tool) is a DSL (Domain-Specific Language) designed to allow human authors to script behaviors for artificial characters. GHOST is inspired by ChatScript in its syntax, it uses [ECAN](https://github.com/opencog/opencog/tree/master/opencog/attention) to guide the context formation and rule discovery processes, [OpenPsi](https://github.com/opencog/opencog/tree/master/opencog/openpsi) for internal dynamics modulation and goal-driven rule (action) selection, as well as various other OpenCog cognitive algorithms for handling a wider range of situations in more flexible and intelligent ways.

## Design Overview

A GHOST rule is essentially an OpenPsi rule (aka psi-rule), which is an `ImplicationLink` that can be expressed as

```
context AND procedure -> goal
```

When a GHOST rule is being created, it will firstly be passed to the parser (`cs-parser.scm`) for syntax checking and preliminary interpretation. Any rules that is not syntactically correct will be rejected at this stage.

The parser will then pass the intermediate interpretations (aka terms) to the translator (`translator.scm`) by calling either `create-rule` / `create-concept` / `create-topic` for creating a psi-rule / concept / topic respectively. Those terms will be converted into OpenCog atoms (defined in `terms.scm`) and stored in the AtomSpace.

Action selector is implemented in `matcher.scm`, which is responsible for selecting a rule that is applicable to a given context in each psi-step. When a textual input is received, it will be converted into a list of WordNodes, wrapped in a `DualLink` and passed to the Recognizer for finding candidates that may satisfy the current context. A full context evaluation will then be done for each of the candidates. Action selector will pick one of them based on their satisfiability and their truth value.

This approach has its own limitation -- it doesn't work well with predicates (e.g. a rule that is triggered only by some sensory input other than words) because right now there is no way to find those rules with efficiency comparable to `DualLink`. As a workaround, those rules will be identified and evaluated in each psi-step, which will consume more computing power than it is needed.

A better yet still experimental approach is to use ECAN to help with rule discovery. When sensory input is received, certain atoms, which can be `WordNodes`, `PredicateNodes`, or some other types of atoms, will be stimulated, results in an increase in their importance value. For now, we focus on the short-term importance (STI) only. The [Importance Diffusion Agent](https://github.com/opencog/opencog/blob/master/opencog/attention/ImportanceDiffusionBase.h) will then diffuse the STI from the atoms being stimulated to their neighboring atoms and, depending on the situation, may bring some actual psi-rules that are potentially applicable to the context to the attentional focus. The action selector in GHOST will look at the attentional focus, pick and evaluate any psi-rules that are in there, and eventually return the most appropriate one that can be executed.

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
u: (what be you name) I forgot; what's YOUR name, sweet wonderful human
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
(ghost-parse "u: (hi robot) Hello human")
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
