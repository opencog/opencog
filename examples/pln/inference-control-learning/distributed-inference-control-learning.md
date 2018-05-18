Distributed Inference Control Learning
======================================

Inference Control Learning in the URE (Unified Rule Engine) is the
process of learning how to efficiently guide reasoning to produce new
knowledge, validate conjectures, etc.

Efficient reasoning is amongst the hardest problems in computer
science. One way to tackle it is to learn how to bias reasoning to be
efficient on problems that matter to us. It will still be generally
extremely innefficient, but as long as these innefficiencies hover
around problem classes of no interest, that is OK.

To do that we

1. Run reasoning over a collection of problems.
2. Record the traces of every decision the URE makes while reasoning.
3. Mine hidden patterns in these traces.
4. Turn these patterns into control rules so that the URE would be
   more efficient at solving these problems the next time.

If the problems the URE is exposed to are sufficiently general, and
the control rules learned sufficiently abstract, then the efficiency
should be transfered to new problems as well.

The catch is that it requires phenomenal computational power. First,
in order to extract valuable information from problems, some must be
solved. If the problems are hard, this alone can be challenging.
Second, in order to attain decent confidences on the control rules, a
large number of problems must be attempted. Third, learning control
rules can itself be costly.

The first two steps, reasonning over a collection problems and
recording their traces, is embarrassingly parallel. Each reasoning
instance can live in its own process/machine, and record traces on its
own local atomspace. Things start getting a little more tricky in the
third step because unarguably many patterns will only surface when
mining data across atomspaces. Then the fourth step can probably take
place in a centralized manner, gathering only knowledge obtained from
the previous step that seem revelant to produce control rules.

So the remaining of that document mostly focuses on step 3, mining
hidden patterns from traces distributed across atomspaces.

Input Data
----------

We only give the representations for backward chaining, which only
minorily differs from forward chaining. Backward chaining traces
represent two types of knowledge.

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
meaning if `<inference>` is expanded from `<premise>` with `<rule>`,
it produces `<new-inference>`.

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
`<TV>` reflects how much we know of it (knowledge of that is not
always certain unless the target gets proven).

Both inferences and rules are represented by `BindLink` (a rule being
in fact an atomic inference)
```
   BindLink
     <variables>
     <clauses>
     <rewrite>
```

Preprocessing
-------------

Let us explain what a control rule is. Its general form is
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
preproof of `<target>`, following some extra `<pattern>`, what are the
odds that `<new-inference>` is a preproof of `<target>`. Where the
odds is captured by the `<TV>` on the implication scope.

Said simply, this represents the odds that applying some rule in a
given inference produces an inference closer to proving the target.

In principle no preproccessing would be required, one only needs to
run this query on the URE to inference control rules. However in
practice any preprocessing that may subsequently reduce the final
effort is worth doing. In particular one can pre-compute all
conjunctions of the form
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

Which is done by merely applying the backward chainer on the target
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
turned into control rules. For that a rule base (usually small) will
need to be run the entire distributed atomspace that contain all
traces over all problems.
