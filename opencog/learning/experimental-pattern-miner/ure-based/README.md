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

### ComposeLink

It seems that using ComposeLink can help. Let's consider the inference
control example

```
  ImplicationScope <rule-TV>
    VariableList
      Variable "$T"
      TypedVariable
        Variable "$A"
        Type "BindLink"
      Variable "$L"
      TypedVariable
        Variable "$B"
        Type "BindLink"
    And
      Execution
        GroundedSchema "expand-and-BIT"
        List
          Variable "$A"
          Variable "$L"
          <rule>
        Variable "$B"
      Evaluation
        Predicate "preproof-of"
        List
          Variable "$A"
          Variable "$T"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$B"
        Variable "$T"
```

We can then write the query

```
Compose
  ImplicationScope <rule-TV>
    VariableList
      Variable "$T"
      TypedVariable
        Variable "$A"
        Type "BindLink"
      Variable "$L"
      TypedVariable
        Variable "$B"
        Type "BindLink"
    And
      Execution
        GroundedSchema "expand-and-BIT"
        List
          Variable "$A"
          Variable "$L"
          <rule>
        Variable "$B"
      Evaluation
        Predicate "preproof-of"
        List
          Variable "$A"
          Variable "$T"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$B"
        Variable "$T"
  List
    Project 0
    Project 1
    Variable "$L-specialized"
    Project 3
```

where `Variable "$L-specialized"` is the variable of the BC query, and
`Project i` is a function projection constructor to let the (i+1)th
variable unchanged. A possible answer would be

```
Compose
  ImplicationScope <rule-TV>
    VariableList
      Variable "$T"
      TypedVariable
        Variable "$A"
        Type "BindLink"
      Variable "$L"
      TypedVariable
        Variable "$B"
        Type "BindLink"
    And
      Execution
        GroundedSchema "expand-and-BIT"
        List
          Variable "$A"
          Variable "$L"
          <rule>
        Variable "$B"
      Evaluation
        Predicate "preproof-of"
        List
          Variable "$A"
          Variable "$T"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$B"
        Variable "$T"
  List
    Project 0
    Project 1
    Lambda
      VariableList
        Variable "$T"
        TypedVariable
          Variable "$A"
          Type "BindLink"
        Variable "$X"
        TypedVariable
          Variable "$B"
          Type "BindLink"
      Inheritance
        Concept "a"
        Variable "$X"
    Project 3
```

Which would correspond to

```
ImplicationScope <rule-TV>
  VariableList
    Variable "$T"
    TypedVariable
      Variable "$A"
      Type "BindLink"
    Variable "$X"
    TypedVariable
      Variable "$B"
      Type "BindLink"
  And
    Execution
      GroundedSchema "expand-and-BIT"
      List
        Variable "$A"
        Inheritance
          Concept "a"
          Variable "$X"
        <rule>
      Variable "$B"
    Evaluation
      Predicate "preproof-of"
      List
        Variable "$A"
        Variable "$T"
  Evaluation
    Predicate "preproof-of"
    List
      Variable "$B"
      Variable "$T"
```
