# Mine Simple Pattern

Mine simple 1-conjunct (a.k.a. 1-gram) pattern.

## Overview

Given the KB
```
(Inheritance
  (Concept "A")
  (Concept "B"))
(Inheritance
  (Concept "A")
  (Concept "C"))
```
find pattern
```
(Lambda
  (Variable "$X")
  (Inheritance
    (Concept "A")
    (Variable "$X")))
```

## Usage

Run `simple.scm` in the guile interpreter

```
guile -l simple.scm
```

or paste scheme commands one by one in guile.

The results should have been stored in the `results` variable, which
can be shown as follows:

```scheme
results
```
