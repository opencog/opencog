Scretch
=======

I see 2 ways go with a URE pattern miner

1. Forward chainer based
2. Backward chainer based

In both cases we need to be able to isolate subpatterns from
superpatterns so that they can be processed individually, either
specialized (forward way), or abstracted (backward way).

## Forward Chainer Based

When the forward chainer is used, we start from an abstract pattern
like

```
Lambda
  X
  X
```

being the most abstract pattern, as it matches the entire atomspace,
and we create more specialized versions. There are 2 ways
specializations can be done

1. composing subpatterns into bigger patterns
2. expanding patterns, replacing leaves by bottom up subtrees

The first way would be for instance

```
Compose
  Type "InheritanceLink"
  Concept "a"
  Lambda
    X
    X
```

which would be in fact equivalent to

```
Lambda
  X
  InheritanceLink
    Concept "a"
    X
```

The fictive link `Compose`, probably badly named [likely constructs
from category theory would provide the right terminology], would take
a pattern, `(Lambda X X)`, and plug it into a bigger pattern starting
with `InheritanceLink`.

The second way would expand `X` in `(Lambda X X)` by for instance
`(InheritanceLink (Concept "a") X)`, obtaining the same

```
Lambda
  X
  InheritanceLink
    Concept "a"
    X
```

The first way is easily ameable to the URE, but does require higher
order constructs, like our badly named `Compose` link.

## Backward Chainer Based

In the backward way, rules express rewriting a specialized pattern
into a more abstract one, such as

```
Lambda
  X
  InheritanceLink
    Concept "a"
    X
```

into

```
Lambda
  X
  X
```

The case is so simple that we can imagine it's not difficult to write
a rule that does that. The problem comes when we want to process
larger patterns such as

```
Lambda
  X
  List
    InheritanceLink
      Concept "a"
      X
    InheritanceLink
      Concept "b"
      X
```

In order to have finite set of URE rules, we need to decompose the
pattern abstraction as to process the subtrees, such as

```
Inheritance
  Concept "a"
  X
```

as fully defined patterns, like

```
Lambda
  X
  Inheritance
    Concept "a"
    X
```

as opposed to a half-defined pattern like above it.
