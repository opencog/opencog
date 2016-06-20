# Overview

Work in progress of a PLN inference to infer that the cost of some
product is less than the cost of another. The facts about the products
would be obtained via the NLP pipeline, and the conclusion would be
passed back to the NLG pipeline to generate the answer to a query.

# AtomSpace representation of the input facts

## (kb.1) Product one costs $100

Output of the NLP sentence "Product one costs $100" -- letting aside
the details about the processing involved to determine that the cost
is in USD.

```atomese
EvaluationLink
  PredicateNode "cost"
  ListLink
    ConceptNode "product one"
    QuantityNode "USD:100"
```

## (kb.2) Product one costs $100

Output of the NLP sentence "Product two costs $200"

```atomese
EvaluationLink
  PredicateNode "cost"
  ListLink
    ConceptNode "product two"
    QuantityNode "USD:200"
```

# Query

"Which one costs less?"

## AtomSpace representation of the answer to that query

```atomese
Evaluation
  Predicate "less"
  List
    ExecutionOutput
      Schema "get-attribute"
      List
        Predicate "cost"
        Concept "product one"
    ExecutionOutput
      Schema "get-attribute"
      List
        Predicate "cost"
        Concept "product two"
```

Given that, as well some knowledge about the uniqueness of cost for
each product, see (kb.6) below, NLG could build the answer

"Product one costs less than product two"

# Atomspace representation of the background knowledge

## (kb.3) Quantity

We are going to assume that

```atomese
QuantityNode "USD:100"
```

stands for

```atomese
QuantityLink
  UnitNode "USD"
  NumberNode 100
```

and that GreaterThanLink, etc, work with Quantity nodes and links in
addition to numbers. Without bringing the full specification of
GreaterThanLink, we will assume our axiomatic scheme can generate
facts on-demand like

```atomese
GreaterThanLink <1,1>
  QuantityNode "USD:200"
  QuantityNode "USD:100"
```

```atomese
GreaterThanLink <0,1>
  QuantityNode "USD:1"
  QuantityNode "USD:100"
```

etc.

## (kb.4) Relate GreaterThanLink to predicate "less"

If Y>X then less(X,Y)

```atomese
Implication <1,1>
  VariableList
    TypedVariable
      Variable "$X"
      Type "QuantityNode"
    TypedVariable
      Variable "$Y"
      Type "QuantityNode"
  GreaterThanLink
    Variable "$Y"
    Variable "$X"
  Evaluation
    Predicate "less"
    List
      Variable "$X"
      Variable "$Y"
```

## (kb.5) Predicate associating unique attributes

If predicate P associates a unique attribute A to individual I, in
other words P is a functional relationship, then the output of
get-attribute(P, I) is A.

```atomese
Implication <1,1>
  VariableList
    TypedVariable
      Variable "$P"
      Type "PredicateNode"
    Variable "$I"
    Variable "$A"
  And
    Evaluation
      Predicate "is-functional"
      Variable "$P"
    Evaluation
      Variable "$P"
      List
        Variable "$I"
        Variable "$A"
  Equal
    ExecutionOutput
      Schema "get-attribute"
      List
        Variable "$P"
        Variable "$I"
    Variable "$A"
```

### (kb.6) Cost associates a unique attribute per individual

```atomese
Evaluation <1,1>
  Predicate "is-functional"
  Predicate "cost"
```

## Compatibility

These higher order facts have been tailored for that demo to make the
inference simpler (as opposed to using more general albeit possibly
more difficult to use facts).

### (kb.7) Equality is compatible with predicate evaluation

```atomese
Implication <1,1>
  VariableList
    TypedVariable
      Variable "$P"
      Type "PredicateNode"
    Variable "$A"
    Variable "$B"
  And
    Equal
      Variable "$A"
      Variable "$B"
    Evaluation
      Variable "$P"
      Variable "$A"
  Evaluation
    Variable "$P"
    Variable "$B"
```

### (kb.8) Equality is compatible with List construction

```atomese
Implication <1,1>
  VariableList
    Variable "$A"
    Variable "$B"
    Variable "$C"
    Variable "$D"
  And
    Equal
      Variable "$A"
      Variable "$B"
    Equal
      Variable "$C"
      Variable "$D"
  Equal
    List
      Variable "$A"
      Variable "$C"
    List
      Variable "$B"
      Variable "$D"
```

# Inference chain

Here the inference chain is built in a forward way. In practice it
would be built backward, from the conclusion associated to the query
back to the initial premises.

## (s.1) 200 USD is greater than 100 USD

From axiomatic scheme (kb.3)

```atomese
GreaterThanLink <1,1>
  QuantityNode "USD:200"
  QuantityNode "USD:100"
```

## (s.2) less(USD:100, USD:200)

Apply modus ponens on implication (kb.4) with premise (s.1)

```atomese
Evaluation <1,1>
  Predicate "less"
  List
    QuantityNode "USD:100"
    QuantityNode "USD:200"
```

## (s.3) The cost of product one is uniquely 100 USD

Apply modus ponens on implication (kb.5) with premises (kb.1) and (kb.6)

```atomese
Equal <1,1>
  ExecutionOutput
    Schema "get-attribute"
    List
      Predicate "cost"
      Concept "product one"
  QuantityNode "USD:100"
```
  
## (s.4) The cost of product two is uniquely 200 USD

Apply modus ponens on implication (kb.5) with premises (kb.2) and (kb.6)

```atomese
Equal <1,1>
  ExecutionOutput
    Schema "get-attribute"
    List
      Predicate "cost"
      Concept "product two"
  QuantityNode "USD:200"
```
  
## (s.5) Pair of costs of product one and two

Apply modules ponens on implication (kb.8) with premises (s.3) and (s.4)

```atomese
Equal <1,1>
  List
    ExecutionOutput
      Schema "get-attribute"
      List
        Predicate "cost"
        Concept "product one"
    ExecutionOutput
      Schema "get-attribute"
      List
        Predicate "cost"
        Concept "product two
  List
    QuantityNode "USD:100"
    QuantityNode "USD:200"
```

## (s.6) Conclusion, the cost of product one is less than the cost of product two

Apply modus ponens to implication (kb.7) with premises (s.5) and (s.2)

```atomese
Evaluation <1,1>
  Predicate "less"
  List
    ExecutionOutput
      Schema "get-attribute"
      List
        Predicate "cost"
        Concept "product one"
    ExecutionOutput
      Schema "get-attribute"
      List
        Predicate "cost"
        Concept "product two"
```
