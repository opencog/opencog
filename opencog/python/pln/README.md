Probabilistic Logic Networks (PLN)
==================================

## Summary

This is a [new PLN design][] that is currently under development. It is still in-progress.

A detailed description of the architecture is available in [algorithm.md](https://github.com/opencog/opencog/blob/master/opencog/python/pln/algorithm.md).

After reading the following high-level tutorial, follow that link to understand the algorithms in detail.

For a detailed technical description of PLN, refer to the following books:

- Goertzel, Ben, Matthew Iklé, Izabela Freire Goertzel, and Ari Heljakka. **Probabilistic logic networks: A comprehensive framework for uncertain inference.** Springer, 2008.

- Goertzel, Ben, Cassio Pennachin, and Nil Geisweiller. **Engineering General Intelligence, Part 2: The CogPrime Architecture for Integrative, Embodied AGI.** Atlantis Press, 2014. (Refer to Section V)

## Tutorial

### Prerequisites:

1) Ensure that all dependencies have been installed. Checkout and build the latest version of OpenCog from the repository.

2) Clone the external-tools repository so that you can use the AtomSpace Viewer:

```git clone https://github.com/opencog/external-tools.git```

### Let's get started:

3) Read the PLN module documentation below to understand the structure of the module.

4) Read **Elements of PLN** and **Forward and backward chaining**:

http://wiki.opencog.org/w/New_PLN_Design,_2013#Elements_of_PLN
http://wiki.opencog.org/w/New_PLN_Design,_2013#Forward_and_backward_chaining

5) Set up **restapi** and specify **deduction_agent.py** and **restapi** in the CogServer configuration as follows:

Install the necessary dependencies for REST API
```cd opencog/python/web/api/```
```sudo ./install_dependencies.sh```
For more information: http://wiki.opencog.org/w/REST_API#Starting_the_REST_API

Before starting the CogServer, the opencog.conf file needs to include the **../opencog/python/pln/examples/deduction** path and the ***../opencog/python/web/api*** path and restapi as a preloaded module:

```PYTHON_EXTENSION_DIRS  = ../opencog/python/web/api, ../opencog/python/pln/examples/deduction```

```PYTHON_PRELOAD = restapi```

6) Start the CogServer

Make sure you're in your opencog's build directory. In its lib directory should be the opencog.conf file which is mandatory for the CogServer.

Run cogserver now with ./opencog/server/cogserver from your build directory.

Switch to a second terminal and type 'telnet localhost 17001'. The opencog shell should now open.

For more information: http://wiki.opencog.org/w/Starting_the_Cogserver

7) Load **deduction_agent.py** in the CogServer

```loadpy deduction_agent```

8) Start the REST API:

Enter the ```help``` command in the CogServer. Confirm that ```restapi.Start``` is listed as a preloaded command.

Enter the ```restapi.Start``` command.

9) Populate the atomspace with some sample contents

Open the scheme shell by typing ```scm```.
Load a scheme file by typing ```(load-scm-from-file "filepath")```
with filepath being, e.g. /home/username/opencog/opencog/python/pln/examples/deduction/atomspace_contents.scm
Exit the scheme shell by typing ```.``` and hitting enter.

10) Load the atomspace in the AtomSpace viewer and zoom in on the subgraph centered around the ConceptNode named Peter. Note what knowledge is represented:

Open ***~/external-tools/AtomViewer/WebContent/atom_viewer.html*** in your browser.
For more information: https://github.com/opencog/external-tools/blob/master/AtomViewer/README.md

11) Start the deduction_agent:

```agents-start deduction_agent.DeductionAgent```

12) Refresh the atomspace in the AtomSpace viewer and zoom in on the subgraph centered around the ConceptNode named Peter. Observe how the subgraph has become more complex with the additional knowledge gained via forward inference.

13) Review the structure of the **ForwardInferenceAgent**, **BackwardInferenceAgent** and **InferenceAgent** implementations in **agents.py**

## Documentation

### PLN Examples in the CogServer

For example code, see the **opencog/python/pln/examples** folder.

Currently, the following examples are available:
-   **deduction** - illustrates how to create a forward chaining agent utilizing **deduction_agent.py**

### Setup

The main intended usage of PLN is within a MindAgent. The high-level steps are:

-   **Create an agent that implements a PLN chainer**
-   **Ensure that the directory path of the agent is in PYTHON_EXTENSION_DIRS in the cogserver configuration file**
-   **Load atoms into the atomspace**
-   **Load the agent**
-   **Start the agent**

Optionally, you can also load the Attention Allocation system to guide the dynamics of the inference process.

The activity of the PLN and Attention Allocation systems can be monitored in real-time by using the AtomSpaceSubscriber tool in the external-tools repository, which demonstrates how to subscribe to JSON-formatted updates.

### PLN Development/testing example in Python without the CogServer

Although the primary use case is to use PLN within a MindAgent loaded into the CogServer, coupled with the Attention Allocation system, for development and testing purposes it can be useful to interact directly with the PLN classes in a standalone Python environment.

A complete example is available in: **opencog/python/pln/examples/deduction_example.py**, which utilizes an agent defined in **deduction_agent.py**.

### PLN Agent example:
Define an initial [atomspace][] object populated with Atoms.

```
from opencog.atomspace import *
atomspace = AtomSpace()
animal = atomspace.add_node(types.ConceptNode, 'animal', TruthValue(.1, TruthValue().confidence_to_count(.9)))
bird = atomspace.add_node(types.ConceptNode, 'bird', TruthValue(.01, TruthValue().confidence_to_count(.9)))
swan = atomspace.add_node(types.ConceptNode, 'swan', TruthValue(.001, TruthValue().confidence_to_count(.9)))
swan_bird = atomspace.add_link(types.InheritanceLink, [swan, bird], TruthValue(0.5, TruthValue().confidence_to_count(.5)))
bird_animal = atomspace.add_link(types.InheritanceLink, [bird, animal], TruthValue(0.5, TruthValue().confidence_to_count(.5)))
```

Then, proceed to load PLN as follows:
```
from pln.chainers import Chainer
from pln.rules import *
```
##### Create a chainer
    chainer = Chainer(atomspace)

##### Choose what rules and link types you want to include in the Chainer
```
link_type = types.InheritanceLink
rule = DeductionRule(chainer, link_type)
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
[InheritanceLink `<swan,bird>` 0.5 0.50000000]
[InheritanceLink `<bird,animal>` 0.5 0.50000000]
[InheritanceLink `<swan,animal>` 0.14899 0.50000000]
```
You can also inspect the detailed history:
    print vars(chainer.history_index)

### Example using the predefined ForwardInferenceAgent

Now that you understand how the previous steps work, you can also
implement them using the predefined ForwardInferenceAgent [MindAgent][].

The previous steps are already encapsulated inside the
[ForwardInferenceAgent class][], which defines a **run** method.
```
forward_inference_agent = pln.ForwardInferenceAgent()
forward_inference_agent.run(atomspace)
```
This is also the pattern to use to activate PLN inside a running
[CogServer][]. After loading the [MindAgent][], it will be called at
each cycle.

### Overview of the PLN architecture

#### Chainer

The [Chainer class][] implements **forward\_step** and
**backward\_step** methods.

#### Rules

You need to add specific [rules][] to the **Chainer class**, which are
implemented by classes that derive from the [Rule class][].

The add\_rule method takes a chainer and a link type as parameters.

You can view the list of implemented rules in the **rules** folder.

After performing [inference][], it stores the results in the **trails**
and **history\_index** members.

An internal method randomly selects which rule to apply in a given
inference step from the available rules in the **\_select\_rule** method
of the [AbstractChainer class][].

#### Inference History

Inference steps are recorded as ExecutionLinks, as implemented in the **AtomSpaceBasedInferenceHistory** class in **chainers.py**.

The format is:

```
ExecutionLink
    rule
    ListLink
    ListLink inputs
    ListLink outputs
```

They are not currently encapsulated in any sort of context in the atomspace, so you have to search for them by link type. For example:

```
(cog-get-atoms 'ExecutionLink)
```

The inference history is also recorded in the **history** attribute of the chainer. Given an instance of a **chainer** object, you can obtain the inference history as follows:

```
chainer.history.get_history()
```

Example instance of an application from the inference history:

```
(ExecutionLink (stv 1.000000 0.000000)
  (GroundedSchemaNode "ModusPonensRule")
  (ListLink (stv 1.000000 0.000000)
    (ListLink (stv 1.000000 0.000000)
      (ImplicationLink (stv 0.500000 1.000000)
        (EvaluationLink (stv 1.000000 0.000000)
          (PredicateNode "smokes")
          (ListLink (stv 1.000000 0.000000)
            (VariableNode "$X")))
        (EvaluationLink (stv 1.000000 0.000000)
          (PredicateNode "cancer")
          (ListLink (stv 1.000000 0.000000)
            (VariableNode "$X"))))
      (EvaluationLink (stv 0.632456 1.000000)
        (PredicateNode "smokes")
        (ListLink (stv 1.000000 0.000000)
          (ConceptNode "Bob"))))
    (ListLink (stv 1.000000 0.000000)
      (EvaluationLink (stv 0.389737 1.000000)
        (PredicateNode "cancer")
        (ListLink (stv 1.000000 0.000000)
          (ConceptNode "Bob"))))))
```

#### Formulas

Each rule is associated with a formula, as described [here][rules]. The
formula determines the truth value of the inference output.

The formulas are defined in **formulas.py** in the **rules** folder.

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

  [AtomSpace]: http://wiki.opencog.org/w/Atomspace
  [InheritanceLink]: http://wiki.opencog.org/w/InheritanceLink
  [MindAgent]: http://wiki.opencog.org/w/Python#MindAgents_in_Python
  [ForwardInferenceAgent class]: http://buildbot.opencog.org/doxygen/d3/d06/classpython_1_1pln_1_1agents_1_1ForwardInferenceAgent.html
  [CogServer]: http://wiki.opencog.org/w/CogServer
  [Chainer class]: http://buildbot.opencog.org/doxygen/db/d69/classpython_1_1pln_1_1chainers_1_1Chainer.html

  [rules]: http://wiki.opencog.org/w/New_PLN_Design,_2013#Rules_and_formulas
  [Rule class]: http://buildbot.opencog.org/doxygen/d8/d78/classpython_1_1pln_1_1rules_1_1rules_1_1Rule.html
  [inference]: http://wiki.opencog.org/w/New_PLN_Design,_2013#Forward_and_backward_chaining
  [AbstractChainer class]: http://buildbot.opencog.org/doxygen/d4/d21/classpython_1_1pln_1_1chainers_1_1AbstractChainer.html

  [Chainer class]: http://buildbot.opencog.org/doxygen/db/d69/classpython_1_1pln_1_1chainers_1_1Chainer.html
  [AbstractChainer class]: http://buildbot.opencog.org/doxygen/d4/d21/classpython_1_1pln_1_1chainers_1_1AbstractChainer.html
  [Logic class]: http://buildbot.opencog.org/doxygen/d3/d02/classpython_1_1pln_1_1logic_1_1Logic.html
  [Attention Allocation]: http://wiki.opencog.org/w/ECAN
  [here]: http://wiki.opencog.org/w/New_PLN_Implementation_Guide,_2013#STEP_1:_BASIC_FIRST-ORDER_PLN
  [1]: http://wiki.opencog.org/w/New_PLN_Design,_2013#The_Atomspace_AttentionalFocus_as_PLN.27s_Working_Memory
  [STI]: http://wiki.opencog.org/w/Short-Term_Importance
  [this mailing list thread]: https://groups.google.com/d/msg/opencog/uA5Ig_wJaT4/1eEsslH0WE8J
  [ImportanceUpdatingAgent]: http://wiki.opencog.org/w/ImportanceUpdatingAgent
