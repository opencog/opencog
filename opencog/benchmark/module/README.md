Benchmark Module
================

***libbenchmark.so***

Runs a performance benchmark on the AtomSpace by creating a fully connected
graph with bidirectional directed edges (requires n^2 - n edges) with specified
concurrency arguments.

Having an integrated benchmark that you can load from the CogServer shell is
useful because it allows you to measure the performance impact of additional
modules that are added to the system and to measure various configuration
scenarios.

Load in the CogServer by adding **opencog/benchmark/module/libbenchmark.so** to
the **MODULES** parameter in **opencog.conf**.

- Invoked from the CogServer shell. Syntax:

```
benchmark-fully-connected OPTION COUNT THREADS
```

where OPTION is concurrent', or 'reset', COUNT is an integer number of nodes,
and THREADS is an integer number of threads.

The nodes are of type ConceptNode, and the links are of type AsymmetricHebbianLink.

- If no arguments are specified, defaults to:

```
benchmark-fully-connected concurrent 500 2
```

indicating multithreaded execution with 500 nodes and 2 threads.

- The 'concurrent' option uses a multithreaded version of for_each from the GNU libstdc++ parallel mode OpenMP library
- The 'reset' option will delete all the ConceptNodes in the AtomSpace along with their incoming sets.

Additional benchmarks can be added to this module later.
