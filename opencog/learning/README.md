OpenCog Learning
================

Collection of learning algorithms, mostly operating on the AtomSpace
but not only. Here is a brief description of each folder

* `clustering` is supposed to regroup clustering algorithms but for
  now only contains an eigenvalue solver, BLOPEX for Block Locally
  Optimal Preconditioned Eigenvalue Xolvers.
* `dimensionalembedding` implements a method of dimensional embedding
  of the atomspace using the Harel-Koren algorithm.
* `miner` implements a rule-engine based pattern miner algorithm for
  the AtomSpace. Comprehensive but slow.
* `pattern-index` implements Andre Senna prototype pattern indexing
  for the AtomSpace, used for fast pattern matching and mining.
* `PatternMiner` implements Shujing Ke pattern miner algorithm for the
  AtomSpace, less comprehensive than `miner` but faster.
* `statistics` more or less generic statistical functions. Should
  probably be moved somewhere else, or maybe better be replaced by
  boost.accumulators.
