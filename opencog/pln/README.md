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
[URE](https://github.com/opencog/atomspace/tree/master/opencog/ure)
where PLN is one or a few specific rule bases, configured with scheme
function and scheme rules.

That folder contains

1. A set of scripts to easily configure the rule engine to utilize PLN
   rule bases.
2. A set of PLN rules, under the subfolder `rules`.
3. A set of PLN meta-rules, rules producing rules, under the subfolder
   `meta-rules`.
4. A set of higher level facts, often algebraic properties like
   symmetry, commutativity, etc, under the subfolder `facts`.

## Status

PLN is a work in progress. Rules are often crudely implemented,
usually for the sake of efficiency while allowing preliminary
experimentations. In particular

1. Many rules are only compatible with the forward chainer, not the
   backward chainer. That is because backward chaining requires that
   the conclusion pattern be statically defined, which makes some
   rules more difficult or less efficient to implement.
2. Confidence calculation is usually very crude, that is because it
   requires some forms of numerical integration to handle second order
   probabilities, which is costly and thus has been neglected so far.
3. Strength calculations are rather approximate too (though better
   than confidence) because ultimately their results depends not only
   on premises strengths but also their confidences.

## Usage

PLN has a scheme module containing various helpers to load rules and
run PLN inferences.

### Configure

First, the module must be loaded

```scheme
(use-modules (opencog pln))
```

Then a PLN rule-base must be loaded

```scheme
(pln-load)
```

At the moment only one rule-base is provided, in the future that same
command will likely accept optional arguments to load subsets or
supersets of PLN.

The rules are loaded in an auxilary atomspace in order not to pollute
the current atomspace. That auxilary atomspace can be accessed via the
`pln-atomspace` variable. In addition the `pln` modules offers helpers
to display its content without having to switch to it

```scheme
(pln-prt-atomspace)
```

To simply list its rule names and weights

```scheme
(pln-list-rules)
```

By default all rules have a default TV as weight, corresponding to a
flat second order distribution (as of today). One may change the
weights as follows

```scheme
(pln-set-rule-tv! rule-name tv)
```

For instance

```scheme
(pln-set-rule-tv! (DefinedSchemaNode "deduction-implication-rule") (stv 0.7 0.2))
```

sets the weight of the deduction implication rule to `(stv 0.7 0.2)`.

### Call Chainers

To call the forward chainer on a given source, simply type

```scheme
(pln-fc source)
```

Likewise to call the backward chainer on a given target, type

```scheme
(pln-bc target)
```

Numerous options can be used, for more information see

```scheme
(help pln-fc)
```

and

```scheme
(help pln-bc)
```

### Examples

PLN examples can be found under the `examples/pln` directory. In
particular for an example using the PLN module see
`examples/pln/good-songs`.
