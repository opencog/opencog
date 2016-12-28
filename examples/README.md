
                  OpenCog Examples
                  ----------------

This directory contains various examples that demonstrate various tasks.
Be sure to explore the github.com/opencog/atomspace/examples directory
first, as that provides most of the simpler examples, as well as showing
how to use the core infrastructure.  The examples here domonstrate
mostly higher-level systems.


python          - Python usage examples.  Most of these are probably
                  obsolete, or mis-categorized, or unmaintained. Needs
                  cleanup.

sureal          - Demonstrates how to use the Surface Realization
                  system, for generating grammatically valid English
                  language sentences from abstract conceptual
                  diagrams.

openpsi        - An example implementation of openpsi to control a simple
                  agent.

Stale Examples:
---------------
The following examples illustrate subsystems that are not currently
maintained, and may be broken or non-operational.

conceptual_blending - Blending together of similar concepts.

hopfield        - Provides a toy example for Economic Attention
                  Allocation Networks, emulating a type of associative
                  memory neural network called a 'Hopfield network'.

visual_demos    - A number of demos to visually represent the dynamics
                  of OpenCog.

Deprecated examples:
--------------------
The strategic direction for opencog is to move away from the current
cogserver + modules + agents design, and to encourage coding entirely
in either scheme and python.   Thus, please reconsider creating new
agents using these deprecated C++ interfaces.

module          - Provides a template for creating a custom module.
agents          - Provides a template for creating a custom agent.
