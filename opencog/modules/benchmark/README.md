Benchmark Module
================

**Runs a performance benchmark on the AtomSpace by creating a fully connected**
**graph with bidirectional directed edges (requires n^2 - n edges) with specified**
**concurrency arguments.**

***Module Name:*** *libbenchmark.so*

An integrated benchmark that you can load from the CogServer shell is useful 
because it allows you to measure the performance impact of additional modules 
that are added to the system and to measure various configuration scenarios.

You can set up your environment in any desired way and load any desired modules
before running the test.

---

#### Instructions

1. Load in the CogServer by adding ```opencog/benchmark/module/libbenchmark.so``` to the ```MODULES``` parameter in ```opencog.conf```.

2. Invoke from the CogServer shell. 

    ***Syntax:***  *benchmark-fully-connected OPTION*

    where **OPTION** is: ```concurrent``` or ```reset``` or ```sti```

    **OPTION** choices:
    - **concurrent**
    
        ***Syntax:*** *benchmark-fully-connected concurrent COUNT THREADS*
    
        Tests concurrent creation of nodes and links using the addAtom operation to form a fully connected graph of n nodes and (n^2 - n) edges where n=COUNT and the number of threads equals THREADS. The nodes are of type ConceptNode, and the links are of type AsymmetricHebbianLink.
        
        Example: ```benchmark-fully-connected concurrent 500 2```
        
    - **sti**
    
        ***Syntax:*** ```benchmark-fully-connected sti```
        
        Updates the STI of every atom (nodes and links) in the atomspace to a random value.
        
    - **reset**
    
        ***Syntax:*** ```benchmark-fully-connected reset```
        
        Delete all the ConceptNodes in the AtomSpace along with their incoming sets.
        


#### Default Parameters
If no arguments are specified, running ```benchmark-fully-connected``` will default to:

```benchmark-fully-connected concurrent 500 2```

indicating multithreaded execution to create a fully connected graph with 500 nodes and 249500 edges using 2 threads.

#### Extensions
The current benchmarks are focused on adding atoms and updating their STI values, primarily for testing the performance impact of changes to the ECAN attention allocation system. Additional benchmarks can be added to this module as needed.

#### Example Benchmarking Session

```
opencog> benchmark-fully-connected reset
All ConceptNodes and their incoming sets deleted.
opencog> benchmark-fully-connected concurrent 500 2
Fully connected graph of 500 total nodes including 500 new nodes.
Wall clock time: 14 seconds
CPU clock time: 19 seconds
Number of threads used: 2
opencog> benchmark-fully-connected sti
250524 atoms updated with random STI values.
Wall clock time: 1 seconds
CPU clock time: 1 seconds
Number of threads used: 2
opencog>
```
