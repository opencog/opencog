# GHOST

GHOST (General Holistic Organism Scripting Tool) is a DSL (Domain-Specific Language) designed to allow human authors to script behaviors for artificial characters. GHOST is inspired by ChatScript in its syntax, it uses [ECAN](https://github.com/opencog/opencog/tree/master/opencog/attention) to guide the context formation and rule discovery processes, [OpenPsi](https://github.com/opencog/opencog/tree/master/opencog/openpsi) for internal dynamics modulation and goal-driven rule (action) selection, as well as various other OpenCog cognitive algorithms for handling a wider range of situations in more flexible and intelligent ways.

## Design Overview

A GHOST rule is essentially an OpenPsi rule (aka psi-rule), which is an `ImplicationLink` that can be expressed as

```
context AND procedure -> goal
```

When a GHOST rule is being created, it will firstly be passed to the parser (`cs-parser.scm`) for syntax checking and preliminary interpretation. Any rules that is not syntactically correct or with unsupported features will be rejected at this stage.

The parser will then pass the intermediate interpretations (aka terms) to the translator (`translator.scm`) by calling either `create-rule` / `create-concept` / `create-topic` for creating a psi-rule / concept / topic respectively. Those terms will be converted into OpenCog atoms (defined in `terms.scm`) and stored in the AtomSpace.

Action selector is implemented in `matcher.scm`, which is responsible for selecting a rule that is applicable to a given context in each psi-step. ECAN is used to help with rule discovery. When sensory input is received, certain atoms, which can be `WordNodes`, `PredicateNodes`, or some other types of atoms, will be stimulated, results in an increase in their importance value. For now, we focus on the short-term importance (STI) only. The [Importance Diffusion Agent](https://github.com/opencog/opencog/blob/master/opencog/attention/ImportanceDiffusionBase.h) will then diffuse the STI from the atoms being stimulated to their neighboring atoms and, depending on the situation, may bring some actual psi-rules that are potentially applicable to the context to the attentional focus. The action selector in GHOST will look at the attentional focus, pick and evaluate any psi-rules that are in there, and eventually return the most appropriate one that can be executed.

A rule that has been triggered will not be triggered again unless you `^keep()` it (see below), but even if you do, it still won't be triggered again within the refractory period. The default refractory period is 1 second.

## Syntax

The syntax of GHOST rules is modeled heavily on [ChatScript](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#rules). However, GHOST uses several ChatScript features for different purposes than they are normally used in ChatScript; and also contains some additional features.

Here is a list of features that are fully supported in GHOST:
- [Word/Lemma](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#canonization)
- [Phrase](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#proper-names)
- [Concept](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#concepts)
- [Choice](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#choices--)
  - Currently predicates (functions) are not supported, only accept word, lemma, phrase, and concepts.
- [Optional](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#optional-words--)
  - Currently predicates (functions) are not supported, only accept word, lemma, phrase, and concepts.
- [Indefinite Wildcard](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#simple-indefinite-wildcards-)
- [Precise Wildcard](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#precise-wildcards-n)
- [Range-restricted Wildcard](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#range-restricted-wildcards-n)
- [Variable](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#_-match-variables)
- [User Variable](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#user_variables)
- [Sentence Boundary](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#sentence-boundaries--and-)
- [Negation](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#not--and-notnot-)
  - Currently predicates (functions) are not supported, only accept word, lemma, phrase, and concepts.
- [Function](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Advanced-User-Manual.md#functions)
  - Currently only Scheme functions are accepted, can support other languages if needed.
  - Should be in the public interface, e.g. use `define-public` when defining the function.
  - There are several build-in functions that are available:
    - `reuse`, to reuse the action of another rule, e.g.
      - `^reuse(some_label)` will reuse the action of another rule with a label named "some_label". It's recommended to use a unique label for each of the rules in the rulebase, `topic.label` is not supported.
      - Once triggered, the rule being reused will also be considered as triggered, so it will not be triggered again unless you `^keep()` it.
      - Note, currently reusing a rule with local variables in the action of the rule is not supported, but user variables are fine.
    - `keep`, to keep the rule in the system so that it can be selected and executed more than once, this can be used at topic level too.
- [Unordered Matching](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#unordered-matching--)

There are different types of rules in ChatScript -- responders (`u:` `s:` `?:`) and gambits (`r:` `t:`). Currently they are handled without distinction, except for questions (`?:`) which will only be triggered if the input is a question.

[Rejoinder](https://github.com/bwilcox-1234/ChatScript/blob/master/WIKI/ChatScript-Basic-User-Manual.md#fast-overview-of-topic-files-top) (`a:` to `q:`) is also supported. Rejoinders have a higher priority than responders and gambits so it will always be selected if it satisfies the context. If more than one rejoinders satisfy the current context, the one that's defined first will always be selected.

The action selection in GHOST is goal-driven, so all of the GHOST rules should be linked to one or more goals. You can link more than one goal to a rule, just like defining concepts, use a space to separate them. The value assigned to a goal will affect the strength of the rules (`ImplicationLinks`) linked to that goal.

There are two ways of creating goals,
1) Top level goal(s)

```
goal: (please_user=0.8)
```

In this case, all the rules created after it will be having the same goal and the same weight, until another top level goal or the end of file is reached.

It's also possible to create a list of rules that are ordered:

```
ordered-goals: (please_user=0.8)
```

The rules being created under a `ordered-goals` will have a different weight, based on the order of creation. The relationship between the order and the weight forms a geometric sequence with a factor of 0.5.

For example, if there are five rules under the above `please_user=0.8` goal, the first rule will have a weight of 0.4, the second one will have 0.2, the third one will have 0.1, and so on. The sum of the weights will get closer to the weight of the top level goal (0.8) if more rules are created under it.

Additionally, when a rule in a sequence is triggered, it will have a lower STI while the next rules in the same sequence will receive a boost in STI, so as to increase the chance of being selected in the next cycle.

Updated May 2018:
Looks like the above methods of biasing the rules to be triggered in the defined order are not enough to give the behavior that one would expect, so an extra condition has been added to the context of the rules to make sure the rule will be triggered only if the previous rules has been triggered under the same ordered-goal.

Updated Jun 2018:
Another experimantal feature has been added -- to select rules based on the pattern specificity, i.e. the more specific rule will always be preferred to less specific one. For example, if there are two rules that can potentially be selected, `(how are you)` and `(how are *)`, then `(how are you)` will be selected. You can do `(ghost-set-specificity-based-as #f)` to turn it off.

2) Rule level goal(s)

```
#goal: (novelty=0.67 please_user=0.4)
u: (what be you name) I forgot; what's YOUR name, sweet wonderful human
```

In this case, the goals will only be linked to the rule created immediately after it. Top level goals will also be linked to the rule if there are any. A top level goal will be overwritten by a rule level goal if the same goal is defined.

There is also an urge associated with a goal. The urge of a goal is 1 (maximum) by default. The default urge can be changed, and it should be done before creating the goal, for example:

```
urge: (please_user=1 novelty=0.5)

goal: (please_user=0.9)
  ; ... rules under the please_user goal ...

goal: (novelty=0.9)
  ; ... rules under the novelty goal ...
```

Basic examples of how to use GHOST is available [HERE](https://github.com/opencog/opencog/blob/master/examples/ghost/basic.scm)

## How To Run

It's recommended to run with [ghost_bridge](https://github.com/opencog/ghost_bridge). GHOST will be started automatically when you run `ghost_bridge`.

On the other hand, if you want to run GHOST alone, follow the steps below:

1) Start the [RelEx server](https://github.com/opencog/relex#opencog-serversh).

   Note: You will need to do `(set-relex-server-host)` if you are running it via Docker.

2) Start the CogServer, e.g.

```
opencog/build/opencog/cogserver/server/cogserver
```

3) Connect to the CogServer in a new terminal, e.g.

```
telnet localhost 17001
```

4) Load the ECAN module

```
loadmodule opencog/build/opencog/attention/libattention.so
```

5) Start ECAN agents

```
agents-start opencog::AFImportanceDiffusionAgent opencog::WAImportanceDiffusionAgent opencog::AFRentCollectionAgent opencog::WARentCollectionAgent
```

6) Load the needed modules

```
(use-modules (opencog)
             (opencog nlp)
             (opencog nlp relex2logic)
             (opencog openpsi)
             (opencog ghost)
             (opencog ghost procedures))
```

7) Before creating any rules, run

```
(ecan-based-ghost-rules #t)
```

Note, rules being created after running this will be slimmer (preferred) and can only work with this ECAN approach. They are NOT backward-compatible with the `test-ghost`.

8) Start authoring, e.g.

```
(ghost-parse "#goal: (novelty=0.24) u: (eat apple) I want an apple")
```

Or use `ghost-parse-files` to parse rule files.

```
(ghost-parse-files "path/to/the/rule/file1" "path/to/the/rule/file2")
```

9) Start GHOST

```
(ghost-run)
```

10) Send some input, e.g.

```
(ghost "I eat apples")
```

The output `[INFO] [say] (I want an apple)` will then be printed on the CogServer.

For experimental purpose, you may change the weights (default = 1) of various parameters being used in action selector, by doing `ghost-set-strength-weight`, `ghost-set-context-weight`, `ghost-set-sti-weight`, and `ghost-set-urge-weight`, for example:

```
(ghost-set-sti-weight 0.5)
(ghost-set-strength-weight 3)
```
