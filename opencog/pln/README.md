# Probabilistic Logic Networks

PLN is a novel conceptual, mathematical and computational approach to uncertain
inference. In order to carry out effective reasoning in real-world
circumstances, AI software must robustly handle uncertainty. However, previous
approaches to uncertain inference do not have the breadth of scope required to
provide an integrated treatment of the disparate forms of cognitively critical
uncertainty as they manifest themselves within the various forms of pragmatic
inference. Going beyond prior probabilistic approaches to uncertain inference,
PLN is able to encompass within uncertain logic such ideas as induction,
abduction, analogy, fuzziness and speculation, and reasoning about time and
causality.

Further details can be found [here](http://wiki.opencog.org/wikihome/index.php/PLN).

## Implementation

The current implementation uses the
[URE](https://github.com/opencog/atomspace/tree/master/opencog/rule-engine)
where PLN is one or a few specific rule bases, configured with scheme
function and scheme rules.

That folder contains

1. A set of scripts to easily configure the rule-engine to utilize PLN
   rule bases.
2. A set of PLN rules, under the subfolder `rules`.
3. A set of PLN meta-rules, rules producing rules, under the subfolder
   `meta-rules`.
2. A set of higher level facts, often algebraic properties like
   symmetry, commutativity, etc, under the subfolder `facts`.

## Status

PLN is a work in progress. Rules are often crudely implemented,
usually for the sake of efficiency to allow preliminary
experimentations. In particular

1. Many rules are only compatible with forward chaining, not the
   backward chainer. That is because backward chaining requires that
   the conclusion pattern be fully defined in statically, which makes
   some rules more difficult or less efficient to implement.
2. Confidence calculation is usually very crude, that is because by
   its nature it requires either some forms of numerical integration,
   which is costly and thus has been neglected so far.

## Usage

PLN contains helpers to configure the URE
