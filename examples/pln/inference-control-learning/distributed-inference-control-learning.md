Distributed Inference Control Learning
======================================

Inference Control Learning in the URE (Unified Rule Engine) is the
process of learning how to efficiently guide reasoning to produce new
knowledge, validate conjectures, etc.

Efficient reasoning is amongst the hardest problems in computer
science. One way to tackle it is to learn how to bias reasoning to be
efficient on problems that matter to us. It will always be generally
innefficient, but as long as these innefficiencies hover around
problem classes of no interest, that is OK.

To do that we

1. Run reasoning over a collection of problems.
2. Record the traces of every decision the URE makes while reasoning.
3. Mine hidden patterns in these traces.
4. Turn these patterns into control rules so that the URE would be
   more efficient at solving these problems the next time.

If the problems the URE is exposed to are sufficiently general, and
the control rules learned sufficiently abstract, then the efficiency
should be transferable to new problems as well.

The catch is that it requires phenomenal computational power. First,
in order to extract valuable information from problems, some must be
solved. If the problems are hard, this alone can be challenging.
Second, in order to attain decent confidences on the control rules, a
large number of problems must be attempted. Third, learning the
control rules can itself be costly.

Fortunately, the first two steps, reasonning over a collection
problems and recording their traces, is embarrassingly parallel. Each
reasoning instance can live in its own process/machine, and record as
well post-process traces on its own local atomspace. Things start
getting a more tricky in the third step because unarguably many
patterns will only surface when mining data across atomspaces. Then
the fourth step can probably take place in a centralized manner,
gathering only knowledge obtained from the previous step that seem
revelant to produce control rules.

The remaining of the document mostly focuses on step 3, mining hidden
patterns from traces distributed across atomspaces.

Input Data
----------

We only give the representations for backward chaining, which only
minorily differs from forward chaining. Backward chaining traces are
represented by two types of knowledge.

1. Connecting inferences to other inferences
```
   ExecutionLink (stv 1 1)
     SchemaNode "URE:BC:expand-inference"
     List
       <inference>
       <premise>
       <rule>
     <new-inference>
```
meaning that `<inference>` is expanded from `<premise>` with `<rule>`
to produce `<new-inference>`.

2. Relating inferences and success
```
   EvaluationLink <TV>
     PredicateNode "URE:BC:preproof-of"
     List
       <inference>
       <target>
```
Here the notion of success is determined by whether a certain
inference is a preproof of some target. That is whether subsequent
expansions may lead to an inference proving the target or not, where
`<TV>` reflects how much we know it (that knowledge is not always
certain unless the target gets proven).

Both inferences and rules are represented by `BindLink` (a rule can be
seen as an atomic inference)
```
   BindLink
     <variables>
     <clauses>
     <rewrite>
```

Preprocessing
-------------

Let us first explain what a control rule is. Its general form is
```
ImplicationScope <TV>
  <vardecl>
  And
    Execution
      GroundedSchema "URE:BC:expand-inference"
      List
        <inference>
        <premise>
        <rule>
      <new-inference>
    Evaluation
      Predicate "URE:BC:preproof-of"
      List
        <inference>
        <target>
    <pattern>
  Evaluation
    Predicate "preproof-of"
    List
      <new-inference>
      <target>
```
expressing that, given that `<inference>` expending from `<premise>`
with `<rule>` produces `<new-inference>`, and `<inference>` is a
preproof of `<target>`, possibly following some extra `<pattern>`, the
odds that `<new-inference>` is a preproof of `<target>` is
`<TV>`.

Said simply, this represents the probability that applying some rule
in a given inference gets us hopefully closer to proving the target.

To do well the distance from the proof should be used instead of the
binary notion of preproof, because some paths are shorter than others,
and we want the inference control to choose the shortest path. But the
notion of preproof is simpler to start with.

In principle no preproccessing would be required, one would only need
to run this query on the URE to infer control rules. However in
practice any preprocessing that may reduce the computation is worth
doing. In particular one can pre-compute all conjunctions of the form
```
  And
    Execution
      GroundedSchema "URE:BC:expand-inference"
      List
        <inference>
        <premise>
        <rule>
      <new-inference>
    Evaluation
      Predicate "URE:BC:preproof-of"
      List
        <inference>
        <target>
```

Which is done by merely applying the backward chainer to the target
above with the [fuzzy conjunction introduction
rule](../../../opencog/pln/rules/propositional/fuzzy-conjunction-introduction.scm). This
can be done on each atomspace separately, thus is embarrassingly
parallel, as even if some inferences are shared across problems,
applying that query on the entire distributed atomspace is not
expected to produce any extra knowledge.

Rule-Engine Queries
-------------------

The pattern miner (which is already a URE process) combined with other
forms of reasonings will be used for uncovering patterns to then be
turned into control rules. For that a rule base (usually small, thus
requiring no distribution) need to be run the entire distributed
atomspace containing all traces over all problems.

Now let's break it down for the simplest possible rules we can learn,
context free control rules expressing the probably of producing a
preproof by expanding any inference from any premise with a given
inference rule, regardless of how that inference and premise look
like. This is equivalent to setting the weight of the inference rule
(or the "second order weight" as confidence is taken into account).

The result is gonna be like
```
ImplicationScope <TV>
  And
    Execution
      GroundedSchema "URE:BC:expand-inference"
      List
        Variable "$inference"
        Variable "$premise"
        <rule>
      Variable "$new-inference"
    Evaluation
      Predicate "URE:BC:preproof-of"
      List
        Variable "$inference"
        Variable "$target"
  Evaluation
    Predicate "preproof-of"
    List
      Variable "$new-inference"
      Variable "$target"
```
where everything is a variable except `<rule>` which hold constant.

Given the previous preprocessing done described in the Section above,
such control rule only require a call a single PLN rule, [conditional
direct
evaluation](../../../opencog/pln/rules/predicate/conditional-direct-evaluation.scm). What
that rule is doing is, given a certain implication, fetch all
instances of its antecedant that are true, let's call that set `A`,
then for each element of `A`, take its valuation (mapping from
variables to values), apply it to the consequent of the implication to
obtain a consequent instance, and if it is true, add it to `C`, then
calculate the resulting TV on the implication as follows
```
TV.strength = |C|/|A|
TV.count = |A|
```
(note that this is not the best possible estimate, this may vary
depending on the prior, but we let aside for sake of simplicity).

It is important to realize that the pattern matchings required to
obtain `A` and `C` need to take place over the entire distributed
AtomSpace of traces.

Of course interesting control rules will require more sophisticated
reasoning schemes such as pattern mining but in the end it will
probably end up requiring the same sort of distributed pattern
matching processing as in this simple case. So I suggest to start with
that.

