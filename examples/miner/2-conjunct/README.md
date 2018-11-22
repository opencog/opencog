# Mine 2-conjunct Pattern

Mine simple 2-conjunct (a.k.a. 2-gram) pattern starting with an
abstract pattern of 2 conjuncts.

## Overview

Given the KB

```
(Inheritance
  (Concept "A")
  (Concept "B")))
(Inheritance
  (Concept "B")
  (Concept "C")))
(Inheritance
  (Concept "D")
  (Concept "E")))
(Inheritance
  (Concept "E")
  (Concept "F")))

```

find pattern

```
(Lambda
  (VariableList
    (Variable "$X")
    (Variable "$Y")
    (Variable "$Z"))
  (And
    (Inheritance
      (Variable "$X")
      (Variable "$Y"))
    (Inheritance
      (Variable "$Y")
      (Variable "$Z"))))
```

## Alternative

Another (usually more efficient) way to mine n-conjunct patterns is to
use incremental conjunction expansion. See the ugly-male-soda-drinker
example for that.

## Usage

Run `2-conjunct.scm` in the guile interpreter

```
guile -l 2-conjunct.scm
```

or paste scheme commends one by one in guile.
