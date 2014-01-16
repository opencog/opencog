PLN Documentation
=================

There is a [new PLN design][] that is currently under development.

The code is located in the [opencog/python/pln directory][]. It is still in-progress.

### Examples

For example code, see the **opencog/python/pln/examples** folder.

Currently, the following examples are available:
-   **deduction** - illustrates how to create a forward chaining agent in **deduction_example.py** utilizing **deduction_agent.py**

### Setup

The main intended usage of PLN is within a MindAgent. The high-level steps are:

-   **Create an agent that implements a PLN chainer**
-   **Ensure that the directory path of the agent is in PYTHON_EXTENSION_DIRS in the cogserver configuration file**
-   **Load atoms into the atomspace**
-   **Load the agent**
-   **Start the agent**

### Example usage

Although the primary intension is to use PLN within a MindAgent, the PLN classes can also be interacted with directly in Python. Here, the basic usage pattern is described.

##### Prerequisites

Define an initial [atomspace][] object populated with Atoms.

For example:
```
from opencog.atomspace import *
atomspace = AtomSpace()
animal = atomspace.add_node(types.ConceptNode, 'animal', TruthValue(.1, .9))
bird = atomspace.add_node(types.ConceptNode, 'bird', TruthValue(.01, .9))
swan = atomspace.add_node(types.ConceptNode, 'swan', TruthValue(.001, .9))
swan_bird = atomspace.add_link(types.InheritanceLink, [swan, bird], TruthValue(0.5, 0.5))
bird_animal = atomspace.add_link(types.InheritanceLink, [bird, animal], TruthValue(0.5, 0.5))
```

Then, proceed to load PLN as follows:
```
from pln.chainers import Chainer
from pln.rules import rules
```
##### Create a chainer
    chainer = Chainer(atomspace)

##### Choose what rules and link types you want to include in the Chainer
```
link_type = types.InheritanceLink
rule = rules.DeductionRule(chainer, link_type)
chainer.add_rule(rule)
```

##### Run the forward chainer
    chainer.forward_step()

#### Results

Check the contents of the [AtomSpace][], and if the forward inference
step was successful, you should see a new [InheritanceLink][].

You can inspect the inference trail:
```
for item in chainer.trails:
    print item
```
```
node[ConceptNode:animal]
node[ConceptNode:bird]
node[ConceptNode:swan]
[InheritanceLink `<swan,bird>` 0.5 0.00062461]
[InheritanceLink `<bird,animal>` 0.5 0.00062461]
[InheritanceLink `<swan,animal>` 0.14899 0.000347102]
```
You can also inspect the detailed history:
    print vars(chainer.history_index)

### Example using the ForwardInferenceAgent

Now that you understand how the previous steps work, you can also
implement them in a [MindAgent][].

The previous steps are already encapsulated inside the
[ForwardInferenceAgent class][], which defines a **run** method.
```
forward_inference_agent = pln.ForwardInferenceAgent()
forward_inference_agent.run(atomspace)
```
This is also the pattern to use to activate PLN inside a running
[CogServer][]. After loading the [MindAgent][], it will be called at
each cycle.

### Explanation of the PLN architecture

#### Chainer

The [Chainer class][] implements **forward\_step** and
**backward\_step** methods.

#### Rules

You need to add specific [rules][] to the **Chainer class**, which are
implemented by classes that derive from the [Rule class][].

The add\_rule method takes a chainer and a link type as parameters.

You can view the list of implemented rules in [rules.py][].

After performing [inference][], it stores the results in the **trails**
and **history\_index** members.

An internal method randomly selects which rule to apply in a given
inference step from the available rules in the **\_select\_rule** method
of the [AbstractChainer class][].

#### Formulas

Each rule is associated with a formula, as described [here][rules]. The
formula determines the truth value of the inference output.

The formulas are defined in [formulas.py][].

For example, the deduction formula is defined as:
```
def deductionSimpleFormula(tvs):
    (sAB, nAB), (sBC, nBC), (_, nA), (sB, nB),  (sC, _) = tvs
    # Temporary filtering fix to make sure that nAB >= nA
    nA = min(nA, nAB)
    sDenominator = low(1 - sB)
    nDenominator = low(nB)
    w1 = DEDUCTION_TERM_WEIGHT # strength
    w2 = 2 - w1 # strength
    sAC = (w1 * sAB * sBC + w2 * (1 - sAB) * (sC - sB * sBC) / sDenominator)    
    nAC = INDEPENDENCE_ASSUMPTION_DISCOUNT * nA * nBC / nDenominator    
    return (sAC, nAC)
```
#### Template Matching

Atoms are selected during inference using template matching.

The call graph for this process starts in the [Chainer class][], calling
the following methods:

-   **forward\_step**
-   **\_apply\_forward**
-   **\_find\_inputs\_recursive** *(the template is defined here)*

The next call is to a method of the [AbstractChainer class][]:

-   **\_select\_one\_matching**

The algorithm is then implemented in the [Logic class][] via the
following call graph:

-   **find**
-   **unify\_together**
-   **unify**
-   **\_unify\_variable** *or* **\_unify\_outgoing**

#### Inference Control

The design specifies that [Attention Allocation][] should play a
prominent role in inference control.

This is referenced [here][] and [here][1] in the design guidelines.

However, the **getAttentionalFocusBoundary** method that is defined in
the [C++ AttentionBank class][] has not yet been integrated with the
Cython bindings.

The **\_selectOne** method in the [Chainer class][] implements a
randomized algorithm weighted by [STI][] to choose an atom for chaining.
More details on a potential future approach to this are available in
[this mailing list thread][].

##### Stimulus

When the \_apply\_rule method of the Chainer class runs, it checks if
\_stimulateAtoms is set.

If set, the \_give\_stimulus method is called, to be applied by the
[ImportanceUpdatingAgent][].

  [new PLN design]: http://wiki.opencog.org/w/New_PLN_Design,_2013
  [opencog/python/pln directory]: https://github.com/opencog/opencog/tree/master/opencog/python/pln
  [atomspace]: http://wiki.opencog.org/w/Python#MindAgents_in_Python

  [AtomSpace]: AtomSpace "wikilink"
  [InheritanceLink]: InheritanceLink "wikilink"
  [MindAgent]: http://wiki.opencog.org/w/Python#MindAgents_in_Python
  [ForwardInferenceAgent class]: http://buildbot.opencog.org/doxygen/d3/d06/classpython_1_1pln_1_1agents_1_1ForwardInferenceAgent.html
  [CogServer]: CogServer "wikilink"
  [Chainer class]: http://buildbot.opencog.org/doxygen/db/d69/classpython_1_1pln_1_1chainers_1_1Chainer.html

  [rules]: http://wiki.opencog.org/w/New_PLN_Design,_2013#Rules_and_formulas
  [Rule class]: http://buildbot.opencog.org/doxygen/d8/d78/classpython_1_1pln_1_1rules_1_1rules_1_1Rule.html
  [rules.py]: http://buildbot.opencog.org/doxygen/db/d7d/pln_2rules_2rules_8py.html
  [inference]: http://wiki.opencog.org/w/New_PLN_Design,_2013#Forward_and_backward_chaining
  [AbstractChainer class]: http://buildbot.opencog.org/doxygen/d4/d21/classpython_1_1pln_1_1chainers_1_1AbstractChainer.html
  [formulas.py]: http://buildbot.opencog.org/doxygen/d1/de2/pln_2formulas_8py.html

  [Chainer class]: http://buildbot.opencog.org/doxygen/db/d69/classpython_1_1pln_1_1chainers_1_1Chainer.html
  [AbstractChainer class]: http://buildbot.opencog.org/doxygen/d4/d21/classpython_1_1pln_1_1chainers_1_1AbstractChainer.html
  [Logic class]: http://buildbot.opencog.org/doxygen/d3/d02/classpython_1_1pln_1_1logic_1_1Logic.html
  [Attention Allocation]: http://wiki.opencog.org/w/ECAN
  [here]: http://wiki.opencog.org/w/New_PLN_Implementation_Guide,_2013#STEP_1:_BASIC_FIRST-ORDER_PLN
  [1]: http://wiki.opencog.org/w/New_PLN_Design,_2013#The_Atomspace_AttentionalFocus_as_PLN.27s_Working_Memory
  [C++ AttentionBank class]: http://buildbot.opencog.org/doxygen/df/d07/classopencog_1_1AttentionBank.html
  [STI]: STI "wikilink"
  [this mailing list thread]: https://groups.google.com/d/msg/opencog/uA5Ig_wJaT4/1eEsslH0WE8J
  [ImportanceUpdatingAgent]: http://wiki.opencog.org/w/ImportanceUpdatingAgent