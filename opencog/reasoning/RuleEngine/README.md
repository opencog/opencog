# RuleEngine

## The PLN implementation so far

The PLN implementation in Python can be found in the [pln directory](../../python/pln).

### Examples

Examples for the current PLN implementation in Python can be found in the
[examples directory](../../python/pln/examples). Among them are the following:

* The original [Socrates demo](../../python/pln/examples/socrates_demo) takes
  input in Scheme and performs reasoning on it using a [MindAgent](../../python/pln/examples/socrates_demo/socrates_agent.py).
  The MindAgent creates and adds rules to a chainer. To display the occurring
  inference steps it is wrapped in a [helper class](../../python/pln/examples/interactive_agent.py).
  
* A [deduction example](../../python/pln/examples/socrates_demo) which specifies
  the input as atoms in Python

* [Additional syllogisms](../../python/pln/examples/relex2logic/syllogisms) that require
  further discussion and will produce further insights. They should ideally be
  processed in a RelEx2Logic-PLN-pipeline
  
* [A demo for backward-chaining](../../python/pln/examples/backward_chaining)
  on an example taken from the Introduction to AI book
  
* [A demo of temporal reasoning](../../python/pln/examples/temporal) integrating
  the [fuzzy Allen Interval algebra](../../python/spatiotemporal).
  
* [A PLN version of the Tuffy MLN "smokes" sample](../../python/pln/examples/tuffy)

* The attempt of a [contextual inference example](../../python/pln/examples/temporal)
  which became out-of-date and has to be refined following discussions summarized
  [here](http://wiki.opencog.org/w/Claims_and_contexts)

### Rules

The PLN rules can be found in the [rules directory](../../python/pln/rules).
Among them are the following:

* [Boolean rules](../../python/pln/rules(boolean_rules.py) create or eliminate
  AndLinks, OrLinks and NotLinks. They haven't been used extensively in PLN
  reasoning so far. They necessitate a guiding heuristic so that links aren't
  produced at random.
* [Context rules](../../python/pln/rules/context_rules.py). They have already
  been implemented in the RuleEngine as three .scm files [here](rules/pln).
* [Direct evaluation rules](../../python/pln/rules/direct_evaluation_rules.py)
  which deal with sets and EvaluationLinks 