Runs a performance benchmark on the AtomSpace by creating a fully connected
graph with bidirectional directed edges (requires n^2 - n edges) with specified
concurrency arguments.

- Invoked from the CogServer shell. Syntax:

```
benchmark-fully-connected OPTION COUNT THREADS
```

where OPTION is 'single', 'concurrent', or 'reset', COUNT is an integer number
of nodes, and THREADS is an integer number of threads.

- If no arguments are specified, defaults to:

```
benchmark-fully-connected concurrent 500 2
```

indicating multithreaded execution with 500 nodes and 2 threads.

- The 'single' option will run synchronously using std::for_each.
- The 'concurrent' option will use a multithreaded version of for_each from the GNU libstdc++ parallel mode OpenMP library

Additional benchmarks can be added to this module later.
