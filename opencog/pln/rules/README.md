# PLN Rule definitions for the URE

Implements PLN rules according to
http://wiki.opencog.org/w/URE_Configuration_Format

## Contents

PLN rules are partitioned into boolean, predicate and term logic.
These terms are used loosely since PLN is rather different than any of
those. The folders are organized as follows

- `propositional` contains rules dealing with `AndLink`, `OrLink` and
  `NotLink` evaluation and introduction. They are fuzzy for the most
  part, contrary to what the name folder seems to suggest.
- `predicate` contains rules for universal and conditional
  instantiation and generalization.
- `term` contains rules dealing with deduction, abduction,
  induction and inversion.
- `wip` contains work-in-progress rules, incomplete or broken. Often
  only compatible with the forward chainer.

## PLN Examples

Multiple examples can be found under

```
<OPENCOG_REPO>/examples/pln
```
