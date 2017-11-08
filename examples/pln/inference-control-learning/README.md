Inference Control Learning
==========================

Experiment with inference control learning. The plan is to

1. run a series of backward inferences,
2. create a corpus of inference traces,
3. Run the pattern miner and other learning algorithms over them,
4. build inference control rules out of these patterns,
5. repeat while reusing these learned inference control rules to speed
   up the backward chainer.

For now it is a toy experiment with a small taylored series over a
small, constant knowledge-base. No ECAN is even necessary.

Knowledge-base
--------------

The knowledge-base is the upper case latin alphabet order. The order
relationship is represented with Inheritance between 2 consecutive
letters.

```
Inheritance (stv 1 1)
  Concept "a" (stv 1/26 1)
  Concept "b" (stv 2/26 1)
...
Inheritance (stv 1 1)
  Concept "y" (stv 25/26 1)
  Concept "z" (stv 26/26 1)
```

The concept strengths, 1/26 to 26/26, are defined to be consistent
with PLN semantics.

On top of that we have the knowledge that A precedes all other
letters.

```
Evaluation (stv 1 1)
  Predicate "alphabetical-order"
  List
    Concept "a"
    Concept "b"
...
Evaluation (stv 1 1)
  Predicate "alphabetical-order"
  List
    Concept "a"
    Concept "z"
```

Combined with a rule to infer that if letter X occurs before Y, then X
inherits Y.

```
ImplicationScope (stv 1 1)
  VariableList
    TypedVariable
      Variable "$X"
      Type "ConceptNode"
    TypedVariable
      Variable "$Y"
      Type "ConceptNode"
  Evaluation
    Predicate "alphabetical-order"
    List
      Variable "$X"
      Variable "$Y"
  Inheritance
    Variable "$X"
    Variable "$Y"
```

TODO add the z pattern, that when is Inheritance X z, then using
conditional instantiation is worthless.

Rule-base
---------

As many PLN rules are included, to make solving problems more
difficult.

Targets
-------

The targets to prove are of the form

```
Inheritance
  X
  Y
```

where X is a letter preceding Y.

Goal
----

The goal of the experiment is to learn control rules. There will be 2
types of useful control rules.

1. Deduction is often useful. That is because chaining an number of
   deductions is enough to prove that any 2 letters are ordered.
2. If the target is of the form Inheritance a X, then conditional
   instantiation is useful. That is because in that case we can use
   this rule over the fact that
   ```
   Evaluation
     alphabetical-order
     List
       a
       X
   ```
   is true for all other letters, combined with the implication that
   is letter X occurs before Y then X inherits from Y.

The first control rule can easily be learned by PLN alone, using
direct evaluation. The second rule however requires something more
sophisticated like the pattern miner to notice that using conditional
instantiation is only fruitful when the first letter of the target is
`a`.

Usage
-----

To run the experiment, launch guile and load `icl.scm`

```
guile -l icl.scm
```

it will automatically run the experiment. After a while you can see
that at the second meta-iteration the number of problems solved has
significantly gone up. In that experiment only context-free control
rules are learned, but that is enough, so no pattern miner is actually
used for now.

The remaining sections explain how it all works.

Experiment High Level Algorithm
-------------------------------

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
   control rules, called ICRi (see Section Control Rule).
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

Generally the control rules will be represent as Cognitive Schematics,
where

* Context:
  + The backward chainer has been queried to prove T
  + Is about to choose what rule to expand and-BIT A from leaf L
  + A is a preproof of T
  + T, A, L fulfill some pattern
* Action:
  + Expand with rule R
* Result:
  + Produce a preproof of T

A preproof is a back-inference-tree prefix (or suffix if you start
from the axioms) of a complete proof of T. In other words it is a
subtree with root T (induced subtree, not bottom-up subtree) of an
inference tree proving T. The reason we use preproof as measure of
success, as opposed to say the likelihood of finding a proof within
the allocated effort, is because it is independent on the difficulty
of the problem. If we can find a preproof it means we're on the right
track no matter what. This may hopefully make it easier to transfer
this knowledge across problem difficulties.

This experiment is simple enough that the context that the backward
chainer has been queried to prove T can be ignored. The context that
it is about to decide what rule to expand can be ignored as well since
it is the universal context of our experiment anyway. The context that
A is a preproof of T seems strange as we have no way to know whether
it is true or not, but we don't need to! It's enough that there is
some positive probability, however small that it may be, to put such
rule in use, the conditional instantiation should ultimately take this
probability into about to estimate the probability of R expanding A
into a preproof of T.

Another context that hasn't been mentioned is the AtomSpace
itself. Indeed an AtomSpace filled with different atoms means
different axiom and rule sets, and thus different inference
controls. Finally, it should be clear that choosing the next inference
rule is only one decision among many others the BC has to make that
are ignored for now.

The simplest, most generic control rule that we can learn expresses
the probability of expanding an and-BIT producing a preproof of any
target. All other control rules are specializations of this rule.

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
        Variable "$A"
        Variable "$L"
        Variable "$R"
      Variable "$B"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$A"
        Variable "$T"
  Evaluation
    Predicate "preproof-of"
    List
      Variable "$B"
      Variable "$T"
```

Such a control rule may already help a bit the Backward Chainer.
Indeed partial instantiation over R will calculate the probability of
a given rule to produce a preproof of any T. To obtain a partial
instantiation over R, R is subtituted by a certain rule, called
`<rule>`, and the TV over the ImplicationScope, called `<rule-TV>`, is
obtained by direct evaluation over the subset of observations of the
corpus of proofs involving `<rule>`. There is a subtlety though, in
the cases where B is not in the trace of subproofs of T we simply
don't know whether or not it could be a subproof, as such we cannot
evaluate its TV as false. Instead we rely on a uncertain prior as
explained in Subsection Record Inference Traces to have uncertain and
partial negative observations.

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
  And
    Execution
      GroundedSchema "expand-and-BIT"
      List
        Variable "$A"
        Variable "$L"
        <rule>
      Variable "$B"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$A"
        Variable "$T"
  Evaluation
    Predicate "preproof-of"
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
  And
    Execution
      GroundedSchema "expand-and-BIT"
      List
        <andbit>
        Variable "$L"
        <rule>
      Variable "$B"
    Evaluation
      Predicate "preproof-of"
      List
        <andbit>
        <target>
  Evaluation
    Predicate "preproof-of"
    List
      Variable "$B"
      <target>
```

where `<target-andbit-rule-TV>` is calculated based on the subset of
observations in the corpus where `<target>`, `<andbit>` and `<rule>`
occurs simultanously. In most cases though such subset will be small
and `<target-andbit-rule-TV>` will have a low confidence. Moreover the
complexity of such a predictor will be high because the complexity of
the instances of T, A, etc, will be counted as well, which will give
it a low prior and in turn will lower the confidence of the produced
term by conditional instantiation (see Section Conditional
Instantiation Confidence).

To go beyond this level-0 meta-learning we need to introduce more
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
        Variable "$A"
        Variable "$L"
        Variable "$R"
      Variable "$B"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$A"
        Variable "$T"
    <pattern>
  Evaluation
    Predicate "preproof-of"
    List
      Variable "$B"
      Variable "$T"
```

where `<pattern>` is a predicate body involving all or some of the
variables T, A, L and R.

The other option is to specialize the preproof and expansion clauses
instead of having an explicit `<pattern>` clause, as we'll see below.

Finally, it's important to realize that B never needs to be
instantiated. In other words, we don't need to try to expand A and
produce B to estimate whether the expansion is worthwhile.

### Example

In this example we craft a rule saying that if the first letter in the
target (or premise leaf) is A, then conditional instantiation almost
surely produces a preproof. Such rule may look like

```
ImplicationScope <TV>
  VariableList
    Variable "$T"  ;; Theorem/target to prove
    TypedVariable  ;; and-BIT to expand
      Variable "$A"
      Type "BindLink"
    Variable "$L"  ;; Leaf from A to expand
    TypedVariable  ;; Resulting and-BIT from the expansion of L from A with rule R
      Variable "$B"
      Type "BindLink"
  And
    Execution
      GroundedSchema "expand-and-BIT"
      List
        Variable "$A"
        Variable "$L"
        <conditional-instantiation>
      Variable "$B"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$A"
        Variable "$T"
    Evaluation
      Predicate "first-letter-is-a"
      Variable "$L"
  Evaluation
    Predicate "preproof-of"
    List
      Variable "$B"
      Variable "$T"
```

You may notice that the inference rule has already been instantiated,
here referred as `<conditional-instantiation>` to keep the example
short. Also the `Predicate "first-letter-is-a"` should be expressed in
terms of patterns mined by the pattern miner.

The other option is to specialize the preproof and expansion clauses
to capture that pattern.

```
ImplicationScope <TV>
  VariableList
    Variable "$T"  ;; Theorem/target to prove
    TypedVariable  ;; and-BIT to expand
      Variable "$A"
      Type "BindLink"
    Variable "$X"  ;; Second letter involved in the leaf to expand
    TypedVariable  ;; Resulting and-BIT from the expansion of L from A with rule R
      Variable "$B"
      Type "BindLink"
  And
    Execution
      GroundedSchema "expand-and-BIT"
      List
        Variable "$A"
        Inheritance
          Concept "a"
          Variable "$X"
        <conditional-instantiation>
      Variable "$B"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$A"
        Variable "$T"
  Evaluation
    Predicate "preproof-of"
    List
      Variable "$B"
      Variable "$T"
```

As you may see, in the expansion clause

```
Variable "$L"
```

has been replaced by

```
Inheritance
  Concept "a"
  Variable "$X"
```

as a substitute for the pattern clause

```
    Evaluation
      Predicate "first-letter-is-a"
      Variable "$L"
```

Learn Inference Control Rules
-----------------------------

### Record Inference Traces

The Backward Chainer can record the following knowledge while
searching for a proof

1. Back-expanding an and-BIT from a certain leaf with certain rule
   produces a new and-BIT
```
   ExecutionLink (stv 1 1)
     SchemaNode "URE:BC:expand-and-BIT"
     List
       <andbit_fcs>
       <bitleaf_body>
       <rule>
     <new_andbit>
```
2. A certain and-BIT proofs a certain target
```
   EvaluationLink <TV>
     PredicateNode "URE:BC:proof-of"
     List
       <andbit_fcs>
       <target_result> <TV>
```

Combining that with axiomatic knowledge about proof and preproof we
can produce a corpus relating and-BITs and targets via the preproof
relationship. For instance if B is a proof of T, we can infer that B
is preproof of T as well.  Moreover if A expands into B, we can infer
that A is a preproof of T.

```
   EvaluationLink <1 1>
     PredicateNode "URE:BC:preproof-of"
     List
       <A>
       <T>
```

Likewise, if no and-BIT C is ever related to T via the preproof
relationship, we can infer that C is unlikely, though still possible,
to be a preproof of T.

```
   EvaluationLink <0.0001 0.01>
     PredicateNode "URE:BC:preproof-of"
     List
       <A>
       <T>
```

The low confidence captures that we are unsure about the prior
itself. These numbers are made up, however it is important that the
confidence remains below 1 so that in case `<A>` happens to be a
preproof of `<T>` this believe can be revised. Over time these numbers
might be autoadjusted via PLN reasoning or such.

These inferences are rather trivial and do not require learning their
inference control to be tractable. See `meta-kb.scm` and `meta-rb.scm`
for their knowledge and the rule bases.

### Produce Inference Control Rules

Given a corpus relating and-BITs and targets via the preproof
relationship we can mine that corpus to obtain control rules, then
combine these control rules to obtain a inference control policy, as
described in the next Section.

We expect the pattern miner will be useful for mining the corpus,
meanwhile we'll experiment with bruteforce implication scope
introduction PLN rule that should be less efficient but easier to
start with. After that, we'll probably want to wrap the pattern miner
in some more specialized and also more efficient implication scope
indroduction rule.

Inference Control Policy
------------------------

Once we have a learned a bunch of control rules we need properly
utilize them to form an Inference Control Policy. Below we describe a
way to combine them and select the next inference rule based on the
results of that combination.

### Combining Inference Control Rules

A variety of control rules may simultaneously apply and we need a way
to combine them.

Let's assume we have k active control rules for inference rule R (a
control rule is active if its context is satisfied).

```
CR1 : C1 & R -> S <TV1>
...
CRk : Ck & R -> S <TVk>
```

where `->` compactly represents an `ImplicationScopeLink` as described
above.

`CRi` expresses that in context `Ci` chosing `R` will produce a
preproof of our final target with truth value `TVi`. `S` stands for
Success. One way of doing that is to use a resource bound variation of
Solomonoff's Universal Operator Induction which, reformulated to fit
our problem, looks like

```
P(S|R) = Sum_i=0^k P(CRi) * CRi(S|R) * Prod_j CRi(Sj|Rj) / nt
```

* `P(S|R)` is the probability that picking up `R` will produce a
  preproof of our final target.
* `P(CRi)` is the prior of `CRi`.
* `CRj(S|R)` is the probability of success upon choosing `R`
  according to `CRi`.
* The last term `Prod_j CRi(Sj|Rj)` is the probability that `CRi`
  explains the corpus of past inferences, the likelihood of `CRi` also
  noted `L(CRi)`.
* `nt` is the normalizing term, defined as
```
nt = Sum_i=0^k P(CRi) * Prod_j CRi(Sj|Rj)
```

Since it is resource bound we cannot consider all computable rules.
Instead we consider the ones we have, obtained by pattern mining or
such.

On top of that, instead of returning a single probability we want to
return a truth value, or more generally a pdf (probability density
function) over probabilities. This is essential to balance
exploitation vs exploration as explained in the next subsection.

Generally, one can do that by building a cumulative distribution
function instead of a probability expectation.

```
CDF_P(S|R)(x) = Sum_i_in_{CRi(S|R)<=x} P(CRi) * Prod_j CRi(Sj|Rj) / nt
```

However our models `CRi` actually calculate TVs (pdfs), not
probabilities, thus `CRi(S|R)<=x` is meaningless. To remedy that we
can split `CRi` into a continuous ensemble of models, each of which
has a probability `p` varying from 0 to 1, indicating the probability
of `S` conditioned by `R`. We can then use this ensemble as extra
models and use the same formula above to calculate the cdf.

```
CDF_P(S|R)(x) = Sum_i P(CRi) Int_0^x Prod_j CRi(Sj|Rj) dp / nt
```

So `P(CRi)` becomes the probability of the ith class of continous
models. Given a infinitesimal model in that class is a mere
probabilty, `CRi(Sj|Rj)`, the probability that `CRi` explains the jth
observation, is in fact `p` if `Rj` yielded success `Sj=1`, `1-p` if
it yielded failure `Sj=0`. Thus `Prod_j CRi(Sj|Rj)` is the binomial
distribution, assuming for now a flat prior (other priors, such as
Jeffrey's are easy consider, as we'll see further below)

```
CDF_P(S|R)(x) = Sum_i P(CRi) Int_0^x p^X*(1-p)^(N-X) dp / nt
```

where `N` is the number of observations and `X` the positive count.

Let define `L` the likelihood of an infinitesimal model with parameter
probability `p`

```
L(p) = p^X*(1-p)^(N-X)
```

Let me recall that the pdf of a TV obtained purely by obversation is a
beta distribution with 

```
alpha = X+1
beta = N-X+1
```

(Section 4.5.1 of the PLN book).

At that stage one can introduce a different prior, like Jeffrey's

```
alpha = X+1/2
beta = N-X+1/2
```

Once `alpha` and `beta` are defined, with whichever prior one chooses,
the following holds

```
L(p) = pdf_beta_distribution_alpha_beta(p) * Beta(alpha, beta)
L(p) = pdf_CRi(p) * Beta(beta, alpha)
```

* `pdf_beta_distribution_alpha_beta` is the pdf of the beta
distribution with parameters `alpa` and `beta`.
* `pdf_CRi` is the pdf of the TV of the ith model.
* `Beta` is the beta function.

We thus obtain the following, almost final, equation

```
CDF_P(S|R)(x) = Sum_i P(CRi) Int_0^x pdf_CRi(p) * Beta(beta, alpha) dp / nt
```

Almost final because, unlike in Universal Operator Induction, our
models are partial, they only compute the probability of the answer,
`Si`, for a subset of questions, all those that satisfy context
`Ci`. Thus the number of positive and total observations for model `i`
are generally not `X` and `N`, but rather

```
X_i<=X
N_i<=N
```

Thus the `alpha` and `beta` parameters must be accordingly
parameterized according to model `i`

```
alpha_i = X_i+1/2
beta_i = N_i-X_i+1/2
```

assuming Jeffrey's prior.

There is no theory, to the best of my knowledge, of a variation of
Universal Operator Induction considering partial operators. To address
that we are going to artificially complete these operators. In
principle, to do that well, one would need to consider all possible
completions, but that is impractical. So instead will only consider
the ones that perfectly explains the out-of-context data. This way the
likelihood doesn't change as the contribution of each out-of-context
datum has a factor of `1`.

```
CDF_P(S|R)(x) = Sum_i P(CRi) Int_0^x pdf_CRi(p) * Beta(alpha_i, beta_i) dp / nt
```

However this is assumption gives an unfair advantage to models with
narrower contexts, and is expected to produce overfitting. So instead
we move the extra complexity to the prior. If the fictive completed
model of `CRi` is called `CRi+`, its prior `P(CRi+)<P(CRi)` has to
account for that extra complexity. We unfortunately do not know it,
but we can establish an upper bound. Indeed the extra complexity
cannot be more than the length of the out-of-context data, because we
can always complete `CRi` with a table mapping the out-of-context
questions to their answers. For now we will use a simplistic heuristic
and assume that the kolmogorov complexity of `CRi+` is

`K(CRi+) = K(CRi) + |D|^(1-c)`

where `c` is a compressability parameter. If `c=0` then `D`, the
out-of-context data, is incompressible, if `c=1` then `D` can be
compressed to a single bit. It is a very crude heuristic of course,
and it requires us to tune a parameter `c`, but it has the advantages
of being simple and computationally lightweight.

The prior of `CRi+` is then

`P(CRi+) = exp(-cpx*K(CRi+))`

where `cpx` is another parameter we'll have to tune. The final
equation of our mixture model is therefore

```
CDF_P(S|R)(x) = Sum_i P(CRi+) Int_0^x pdf_CRi(p) * Beta(alpha_i, beta_i) dp / nt
```

Last remark. This does assume that all observations are certain (based
on perfect sensors). In practice we actually need to consider
imperfect sensors because the negative observations of preproof have
uncertainties, as explained in Subsection Record Inference Traces. It
is expected that we'll have to resort to convolution products, because
the pdf of a random variable equal to the sum of other random
variables is determined by the convolution products of their
pdfs. However it is suspected that the convolution products of 2
beta-distributions is a beta-distribution, which should simplify
things a lot. This is highly speculative of course.

Once we have that we can calculate the TVi of success of each active
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

Thus rewritting the above for any Pi and simplifying a bit

```
Pi = I_0^1 pdfi(p) * Prod_j!=i cdfi(p) dp
```

where `Prod_j!=i fi` is the product of all fi with j from 1 to n,
except i.

If we store the values (assuming we don't already have them stored in
the TV objects) of

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
select our next inference rule.

Maybe it's possible to have all this done by PLN itself by "inverting"
the cognitive schematics to make it say

```
C & G -> A
```

that is, if we are in context `C` and want to fulfill goal `G` then
what is the probability that should be `A` selected. Then merely use
"holistic" instantiation.

Further Remarks
---------------

This is a fairely general proprosal, for instance although only the
rule selection step is considered, it pretty much covers most (if not
all) hard decision points occuring in the Backward Chainer
algorithm. For instance choosing an and-BIT for expansion comes down
to choosing an unexplored preproof. Choosing the next leaf to expand
from, consists of choosing a leaf such that there exists an inference
rule that expands into a preproof. So the control rules can in fact be
re-used, or need little modifications, for other decisional points.

There is however a problem that has been completely left out. How to
characterize the AtomSpace, i.e. the knowledge base. Proof structures
and sizes depends on it. It is expected that we'll have to capture
relevant properties of the knowledge base, which seems hard given how
complex it can be. Maybe dimensional embeding will be shown to be
useful, amonsgt many other techniques. Similarily, how ECAN is gonna
interact synergetically with this is not clear to me at this point.

References
----------

[1] Jan Leike et al - Thompson Sampling is Asymptotically Optimal in
General Environments.
