# RuleEngine

## The PLN implementation so far

The old PLN implementation in Python can be found in the [old pln directory](../../python/pln_old).

### Examples

Examples for the current PLN implementation in Python can be found in the
[examples directory](../../python/pln_old/examples). Among them are the following:

* The original [Socrates demo](../../python/pln_old/examples/socrates_demo) takes
  input in Scheme and performs reasoning on it using a [MindAgent](../../python/pln_old/examples/socrates_demo/socrates_agent.py).
  The MindAgent creates and adds rules to a chainer. To display the occurring
  inference steps it is wrapped in a [helper class](../../python/pln_old/examples/interactive_agent.py).

* A [deduction example](../../python/pln_old/examples/socrates_demo) which specifies
  the input as atoms in Python

* [Additional syllogisms](../../python/pln_old/examples/relex2logic/syllogisms) that require
  further discussion and will produce further insights. They should ideally be
  processed in a RelEx2Logic-PLN-pipeline
  
* [A demo for backward-chaining](../../python/pln_old/examples/backward_chaining)
  on an example taken from the Introduction to AI book
  
* [A demo of temporal reasoning](../../python/pln_old/examples/temporal) integrating
  the [fuzzy Allen Interval algebra](../../python/spatiotemporal).
  
* [A PLN version of the Tuffy MLN "smokes" sample](../../python/pln_old/examples/tuffy)

* An [agent](../../python/pln_old/examples/attentionallocation/socrates_attention_agent.py)
  used for [attentionallocation](../../python/pln_old/examples/attentionallocation).
  For further information see below.

* The attempt of a [contextual inference example](../../python/pln_old/examples/temporal)
  which became out-of-date and has to be refined following discussions summarized
  [here](http://wiki.opencog.org/w/Claims_and_contexts)

### Rules

The PLN rules can be found in the [rules directory](../../python/pln_old/rules).
Among them are the following:

* [Boolean rules](../../python/pln_old/rules/boolean_rules.py) create or eliminate
  AndLinks, OrLinks and NotLinks. They haven't been used extensively in PLN
  reasoning so far. They necessitate a guiding heuristic so that links aren't
  produced at random.

* [Context rules](../../python/pln_old/rules/context_rules.py). They have already
  been implemented in the RuleEngine as three .scm files [here](rules/pln).

* [Direct evaluation rules](../../python/pln_old/rules/direct_evaluation_rules.py)
  deal with sets and EvaluationLinks .

* [Inheritance rules](../../python/pln_old/rules/inheritance_rules.py) reason
  with InheritanceLinks and contain the classic deduction, abduction, induction,
  etc.

* [Predicate rules](../../python/pln_old/rules/predicate_rules.py). This module
  only handles one rule. It should be evaluated if it can be integrated with
  another one, e.g. the direct evaluation rules.

* [Quantifier rules](../../python/pln_old/rules/quantifier_rules.py) implement
  AverageCreationRule and ScholemRule. There are still rules missing for other
  quantifiers, however.

* [Temporal rules](../../python/pln_old/rules/context_rules.py) integrate PLN
  with the the fuzzy Allen Interval algebra. More discussion is in order to
  revise them (see below).(https://github.com/opencog/opencog/pull/934)).

* [Formulas](../../python/pln_old/rules/formulas.py) specify the calculation or
  creation of simple truth values for the PLN rules.


### Attention allocation

Demos for attention allocation can be viewed
[here](../../../../external-tools/attention/client/examples). Both use the
[client interface](../../../../external-tools/attention/client/client.py) and
the [REST API](../../python/web/api). Both employ the Socrates syllogism to
demonstrate reasoning and use the [agent](../../python/pln_old/examples/attentionallocation/socrates_attention_agent.py)
mentioned above.
[Socrates_attention.py](../../../../external-tools/attention/client/examples/socrates_attention.py)
runs the agent in the CogServer while [socrates_demo](../../../../external-tools/attention/client/socrates_demo.py)
runs the agent using Python and uses the CogServer only as an intermediary for
visualization in conjunction with the client interface.


### Temporal reasoning

The temporal rules use obsolete [temporal formulas](../../python/spatiotemporal/temporal_formulas.py).
The implementation of the fuzzy Allen Interval algebra can be found
[here](../../python/spatiotemporal). A relevant pull request can be found
[here](https://github.com/opencog/opencog/pull/934) while the discussion can
be viewed [here](https://groups.google.com/forum/#!topic/opencog/NhWMI4p72UI).
