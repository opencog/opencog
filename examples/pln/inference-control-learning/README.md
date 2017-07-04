Inference Control Learning
==========================

Experiment with inference control learning. The plan is to

1. run a series of backward inferences,
2. create a corpus of successful ones,
3. Run the pattern miner over them,
4. build cognitive schematics out of these patterns,
5. repeat with more inferences passing these cognitive schematics to
   the backward chainer for speeding it up.

For now it is a toy experiment with a small taylored series over a
small, constant knowledge-base. No ECAN is even necessary. The learned
inference control rules should be that double, triple, etc deductions
are useful.

Knowledge-base
--------------

The knowledge-base is the upper case latin alphabet order. The order
relationship is represented with Inheritance between 2 consecutive
letters:

```
Inheritance (stv 1 1)
  Concept "A" (stv 1/26 1)
  Concept "B" (stv 2/26 1)
...
Inheritance (stv 1 1)
  Concept "Y" (stv 25/26 1)
  Concept "Z" (stv 26/26 1)
```

The concept strengths, 1/26 to 26/26, are defined to be consistent
with PLN semantics.

Rule-base
---------

All PLN rules are included, to make solving problems more difficult.

Targets
-------

The targets to prove are of the form

```
Inheritance
  X
  Y
```

where X is a letter preceding Y.

Experiment Algorithm
--------------------

0. 
   1. Randomly generate a set of N problems, called PS, given a random
      seed. Each problem vary only by its target, all other parameters
      such as termination criteria are fixed. Problems have different
      levels of difficulties, determined by the alphabetic distance
      between 2 letters.
   2. Initialize i = 0, the meta-iteration (or iteration) index.
1. Run the BC over each problem pj in PS, j is the problem index.
   Save the logs of each problem in opencog-i-j.log.
2. Store in Si the number of solved problems for this iteration.
3. Given opencog-i-j.log for j in 0 to N-1, build a collection of
   inference control rules, called ICRi (see Setion Inference Control
   Rule).
4. If meta-termination hasn't occured repeat step 1 with passing ICRi
   to the BC.

The idea is to run this for M iterations and see how Si evolves over
time. Here of course given the very simplistic problem set it is
likely that Si would be maxed out at the second iteration, given that
all problems teach the same lesson.

An alternative version is to regenerate PS with a different random
seed at each iteration. Then a low N and high M would mimick more
naturally ongoing meta-learning.

Inference Control Rule
----------------------

In this toy experiment the expected inference control rules should be
that double, triple, etc, deductions are the ones likely to solve the
problems. But generally the inference control rules will be represent
as Cognitive Schematics, where

* Context:
  + The backward chainer has been queried to prove T
  + Is about to choose what rule to expand and-BIT A from leaf L
  + T, A, L fulfill some pattern
* Action:
  + Expand with rule R
* Result:
  + Produce a preproof of T

A preproof is a back-inference-tree prefix (or suffix if you start
from the axioms) of a complete proof of T. In other words it is a
subtree with root T (such as in graph theory, not computer data
structure) of an inference tree proving T. The reason we use preproof
as measure of success, as opposed to say the likelihood of finding a
proof within the allocated effort, is because it is independent on the
difficulty of the problem. If we can produce a preproof it means we're
on the right track no matter. This may hopefully make it easier to
transfer this knowledge across problem difficulties.

This experiment is simple enough that the context that the backward
chainer has been queried to prove T can be ignored. The context that
it is about to decide what rule to expand can be ignored as well since
it is the universal context of our experiment anyway. Another context
that hasn't been mentioned is the AtomSpace itself. Indeed an
AtomSpace filled with different atoms means different axiom and rule
sets, and thus different inference controls. Finally, it should be
clear that choosing the next inference rule is only one decision among
many others the BC has to make that are ignored for now.

The simplest, most generic inference control rule that we can learn
expresses the probability of expanding an and-BIT producing a preproof
of any target. All other inference control rules are specializations
of this rule.

```
ImplicationScope <TV>
  VariableList
    Variable "$T"  ;; Theorem/target to prove
    TypedVariable  ;; and-BIT to expand
      Variable "$A"
      Type "BindLink"
    Variable "$L"  ;; Leaf from A to expand
    Variable "$R"  ;; Rule to expand A from L
    TypedVariable  ;; Resulting and-BIT from the expansion of L from A with rule R
      Variable "$B"
      Type "BindLink"
  Execution
    GroundedSchema "expand-and-BIT"
    List
      "$A"
      "$L"
      "$R"
    "$B"
  Evaluation
    Predicate "preproof"
    List
      Variable "$B"
      Variable "$T"
```

Such an inference control rule may already help a bit the Backward
Chainer. Indeed partial instantiation over R will calculate the
probability of a given rule to produce a preproof of any T. To obtain
a partial instantiation over R, R is subtituted by a certain rule,
called <rule>, and the TV over the ImplicationScope, called <rule-TV>,
is obtained by direct evaluation over the subset of observations of
the corpus of proofs involving <rule>. There is a subtlety though, in
the cases where B is not in the trace of subproofs of T we simply
don't know whether or not it could be a subproof, as such we cannot
evaluate its TV as false. Instead we evaluate these as unknown, which
would correspond to an simple TV with null confidence. So when we
calculate the TV of the rule we need to take all these unknown into
account and they will flatten the associate distribution, even when
the number of instances is very high. TODO: find the exact formula.

```
ImplicationScope <rule-TV>
  VariableList
    Variable "$T"
    TypedVariable
      Variable "$A"
      Type "BindLink"
    Variable "$L"
    TypedVariable
      Variable "$B"
      Type "BindLink"
  Execution
    GroundedSchema "expand-and-BIT"
    List
      "$A"
      "$L"
      <rule>
    "$B"
  Evaluation
    Predicate "preproof"
    List
      Variable "$B"
      Variable "$T"
```

This is a good way to assign the weights of the rules for the next
round of reasoning, but it might not go much further. Indeed partial
instantiation will likely fail for unknown instances of T, A, L or R
because direct evaluation won't be possible. If at least some
instances have been encountered, for instance T and A are known but L
isn't, then we can still come up with some possibly useful partial
instantiation of that rule, such as

```
ImplicationScope <target-andbit-rule-TV>
  VariableList
    Variable "$L"
    TypedVariable
      Variable "$B"
      Type "BindLink"
  Execution
    GroundedSchema "expand-and-BIT"
    List
      <andbit>
      "$L"
      <rule>
    "$B"
  Evaluation
    Predicate "preproof"
    List
      Variable "$B"
      <target>
```

where <target-andbit-rule-TV> is calculated based on the subset of
observations in the corpus where <target>, <andbit> and <rule> occurs
simultanously. In most cases though such subset will be small and
<target-andbit-rule-TV> will have a low confidence. Moreover the
complexity of such a predictor will be high because the complexity of
the instances of T, A, etc, will be counted as well, which will give
it a low prior and in turn will lower the confidence of the produced
term by conditional instantiation (see Section Conditional
Instantiation Confidence).

To go beyond this level-0 meta-learning type we need to introduce more
expressive specializations in order to generalize well when new
instances of T, A, L, etc, are encountered.

Let's give the general form of such specialization

```
ImplicationScope <TV>
  VariableList
    Variable "$T"  ;; Theorem/target to prove
    TypedVariable  ;; and-BIT to expand
      Variable "$A"
      Type "BindLink"
    Variable "$L"  ;; Leaf from A to expand
    Variable "$R"  ;; Rule to expand A from L
    TypedVariable  ;; Resulting and-BIT from the expansion of L from A with rule R
      Variable "$B"
      Type "BindLink"
  And
    Execution
      GroundedSchema "expand-and-BIT"
      List
        "$A"
        "$L"
        "$R"
      "$B"
    <pattern>
  Evaluation
    Predicate "preproof"
    List
      Variable "$B"
      Variable "$T"
```

where <pattern> is a predicate body involving all or some of the
variables T, A, L and R.

TODO: give an example with double deduction.

```
  (ExecutionOutputLink
    (GroundedSchemaNode "scm: bc-deduction-formula") ; [5481509939359570705][1]
    (ListLink
      (InheritanceLink
        (ConceptNode "A") ; [6977206836600634430][1]
        (ConceptNode "D") ; [246112806454457922][1]
      ) ; [9592798904207778024][1]
      (ExecutionOutputLink
        (GroundedSchemaNode "scm: bc-deduction-formula") ; [5481509939359570705][1]
        (ListLink
          (InheritanceLink
            (ConceptNode "A") ; [6977206836600634430][1]
            (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
          ) ; [13444058388333684400][15]
          (InheritanceLink
            (ConceptNode "A") ; [6977206836600634430][1]
            (VariableNode "$B-6229393a") ; [6185394777777469381][15]
          ) ; [15532080875530789483][15]
          (InheritanceLink
            (VariableNode "$B-6229393a") ; [6185394777777469381][15]
            (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
          ) ; [14984376557733565207][15]
        ) ; [18175419943537721797][15]
      ) ; [16724333062125429183][15]
      (InheritanceLink
        (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
        (ConceptNode "D") ; [246112806454457922][1]
      ) ; [16015351290941397556][15]
    ) ; [15046555205905734382][15]
  ) ; [13595468324493441768][15]

[15532080875530789483][15] [14984376557733565207][15]
----------------bc-deduction-formula-----------------
             [13444058388333684400][15] [16015351290941397556][15]
             ----------------bc-deduction-formula-----------------
                           [9592798904207778024][1]

[DEBUG] [URE] Expanded forward chainer strategy:
(BindLink
  (VariableList
    (TypedVariableLink
      (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
      (TypeNode "ConceptNode") ; [3788634541270868382][1]
    ) ; [11809658565253834475][15]
    (TypedVariableLink
      (VariableNode "$B-6229393a") ; [6185394777777469381][15]
      (TypeNode "ConceptNode") ; [3788634541270868382][1]
    ) ; [16150796384774871558][15]
  ) ; [10107478639882222213][15]
  (AndLink
  ) ; [17473895290224991152][15]
  (ExecutionOutputLink
    (GroundedSchemaNode "scm: bc-deduction-formula") ; [5481509939359570705][1]
    (ListLink
      (InheritanceLink
        (ConceptNode "A") ; [6977206836600634430][1]
        (ConceptNode "D") ; [246112806454457922][1]
      ) ; [9592798904207778024][1]
      (ExecutionOutputLink
        (GroundedSchemaNode "scm: bc-deduction-formula") ; [5481509939359570705][1]
        (ListLink
          (InheritanceLink
            (ConceptNode "A") ; [6977206836600634430][1]
            (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
          ) ; [13444058388333684400][15]
          (InheritanceLink
            (ConceptNode "A") ; [6977206836600634430][1]
            (VariableNode "$B-6229393a") ; [6185394777777469381][15]
          ) ; [15532080875530789483][15]
          (InheritanceLink
            (VariableNode "$B-6229393a") ; [6185394777777469381][15]
            (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
          ) ; [14984376557733565207][15]
        ) ; [18175419943537721797][15]
      ) ; [16724333062125429183][15]
      (InheritanceLink
        (VariableNode "$B-6266d6f2") ; [4097372290580364298][15]
        (ConceptNode "D") ; [246112806454457922][1]
      ) ; [16015351290941397556][15]
    ) ; [15046555205905734382][15]
  ) ; [13595468324493441768][15]
) ; [17301981865595416016][15]

[DEBUG] [URE] With inference tree:
```

Learning Inference Control Rules
--------------------------------

TODO: pattern mining, etc.

Inference Control Policy
------------------------

Once we have a learned a bunch of inference control rules we need
properly utilize them to form an Inference Control Policy. Below we
describe a way to combine them and select the next inference rule
based on the results of that combination.

### Combining Inference Control Rules

A variety of inference control rules may simultaneously apply and we
need a way to combine them.

Let's assume we have k valid inference control rules for inference
rule R (an inference control rule is valid if it unifies with the
current intermediary target and have its contexts satisfied.)

```
ICR1 : C1 & R -> S <TV1>
...
ICRk : Ck & R -> S <TVk>
```

where `->` compactly represent an `ImplicationScopeLink` as described
above.

`ICRi` expresses that in context `Ci` chosing `R` will produce a
preproof of our final target with truth values `TVi`. `S` stands for
Success. One way of doing that is to use a resource bound variation of
Solomonoff's Universal Operator Induction which, reformulated to fit
our problem, looks like

```
P(S|R) = Sum_i=0^k P(ICRi) * ICRi(S|R) * Prod_j ICRi(Sj|Rj) / nt
```

* `P(S|R)` is the probability that picking up `R` in this instance
  will produce a preproof of our final target.
* `P(ICRi)` is the prior of `ICRi`.
* `ICRj(S|R)` is the probability of success upon choosing `R`
  according to `ICRi`.
* The last term `Prod_j ICRi(Sj|Rj)` is the probability that `ICRi`
  explains our corpus of past inferences.
* `nt` is the normalizing term, defined as
```
nt = Sum_i=0^n P(ICRi) * Prod_j ICRi(Sj|Rj)
```

Since it is resource bound we cannot consider all computable rules.
Instead we consider the ones we have, obtained by pattern mining or
such.

On top of that, instead of returning a single probability we want to
return a truth value, or more generally a pdf (probability density
function). This is essential to balance exploitation vs exploration as
explained in the next subsection.

Generally, one can do that by building a cumulative distribution
function instead of a probability expectation.

```
CDF_P(S|R)(x) = Sum_i_in_{ICRi(S|R)<=x} P(ICRi) * Prod_j ICRi(Sj|Rj) / nt
```

However our models `ICRi` calculate TVs (thus pdfs), not
probabilities, making `ICRi(S|R)<=x` ill-defined. To remedy that we
can split `ICRi` into a continuous ensemble of models, each of which
with a probability from 0 to 1, not a TV, associated to it. We can
then use this ensemble as extra models and use the same formula above
to calculate the cdf. Luckily it turns out that the TV corresponding
to `ICRi` precisely represents the cdf of `Prod_j ICRi(Sj|Rj)`, so in
fact the cdf of `P(S|R)` is merely a weighted sum of the cdfs of
`ICRi`

```
CDF_P(S|R) = Sum_i=0^n CDF_ICRi(S|R) * P(ICRi)
```

where

```
CDF_ICRi(S|R)(x) = Int_0^x p^X*(1-p)^(N-X) dp
```

`N` is the number of observations and `X` the positive count. Which
corresponds to, up to a multiplicative constant, the second order
distribution representing a TV as defined in Section 4.5.1 of the PLN
book. That is assuming that all observations are certain (based on
perfect sensors). We'll see that we actually do need to consider
imperfect sensors.

TODO: how to properly normalize?

Once we have that we can calculate the TVi of success of each valid
inference rule Ri, either by turning its cdf into a TV or a pdf as it
will be useful below.

### Inference Rule Selection

Given a distribution of success over inference rules

```
R1 <TV1>
...
Rn <TV1>
```

we want to choose the right one. The reasonable choice is to pick the
one with the highest probability of success. However, because these
probabilities are uncertain, the one with the highest estimate might
not be the best. There is also an exploitation vs exploration
trade-off to make.

One way to solve this is to use tournament selection. A k-way
tournament selection selects k candidates uniformly randomly and picks
up the best one. The higher the uncertainty the lower k should
be. When k=1 this amount to uniform random selection, i.e. total
uncertainty, when k=n this amount to picking up the rule with highest
probability, i.e. total certainty.

Unfortunately this method has some problems. First, it doesn't work
well when the probability estimates have very different confidences.
One option is to account for the confidence in the score to be passed
to tournament selection. If s is the probability and c the confidence,
then we may consider for instance s*c instead of s alone. This avoids
to pick rules with large strength but low confidence, which is perfect
for exploitation but bad for exploration.  Second, it doesn't work
well if n is small, which is often the case in rule selection as only
a couple of rules may be valid for a given intermediary target. If
n=2, then k can be either 1, total randomness, or 2, total
determinism.

In order to address that we can estimate the probability distribution
of actions (here inference rules) in a conceptually similar fashion to
using Thompson Sampling for controlling universal agents [1]. Though
instead of sampling the environment we build the probability
distribution of action and sample from it. I claim that this is more
adequate in our case (which can be revisited later on). First, the
horizon is only 1 step ahead so I doubt it is more costly. Second, our
models, in OpenCog, are partial and represent in fact classes of
models (that is how uncertainty is modeled, as explained below).
Third, it helps to clearly seperate the policy from the world model.

Assume we have n rules with TVs tv1, ..., tvn. The idea is to
calculate the probability that a rule is the best based on the pdfs,
called pdf1, ..., pdfn of the TVs. The integration to solve is

```
Pi = I1_0^1 ... In_0^1
     pdf1(p1)*...*pdfn(pn)*1(p1<=pi and ... and pn<=pi)
     dp1 ... dpn
```

where `Pi` is the probability that rule i is equal or better than all
the others. `1(.)` is the function returning 1 iff its expression is
true, 0 otherwise.

Such an integration is computationally expensive, `O(m^n)` where `m`
is the number of bins used to discretize the integral. Fortunately it
can be greatly simplified. First the expression inside `1(.)` merely
limits the range of the integration variables different than
`pi`. Without loss of generality we can assume that this variable is
`p1`, we get

```
P1 = I1_0^1 I2_0_^p1 ... In_0_^p1 pdf1(p1)*...*pdfn(pn) dp1 ... dpn
```

Second, we can factorize each intergral out of the rest as they are
independent of each others, starting by the pdf terms

```
P1 = I1_0^1 pdf1(p1) I2_0_^p1 pdf2(p2) ... In_0_^p1 pdfn(pn) dpn ... dp1
   = I1_0^1 pdf1(p1) * (I2_0_^p1 pdf2(p2) dp2) * ... * (In_0_^p1 pdfn(pn) dpn) dp1
```

Using cdfs this can be simplified into

```
P1 = I1_0^1 pdf1(p1) * cdf2(p1) * ... * cdfn(p1) dp1
```

If we store the values of

```
pdfi(p)
```

and

```
cdfi(p)
```

in two `n*m` tables, for `i` in `[0, n)` and `p` in `(0/m, m/m]`, then
the complexity should be around `O(n*M)`, all rules considered. In the
end we end up with a distribution of actions according to which we can
pick up our next inference rule.

References
----------

[1] Jan Leike et al - Thompson Sampling is Asymptotically Optimal in
General Environments.
