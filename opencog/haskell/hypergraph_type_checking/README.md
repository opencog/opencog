# Attempts to use Haskell type checker for hypergraph type checking

This folder contains one or more Haskell files to experiment with
hypergraph functional type checking, as firstly described in
https://github.com/opencog/opencog/issues/1311

## Experiment with 'data'

```
experiment_with_data.hs
```

is a proposal using the Haskell type construct 'data'.

You may load it in the Haskell interpreter

```
ghci experiment_with_data.hs
```

and check the types of h1, h2 and h3

```
*Main> :t h1
h1 :: Concept
*Main> :t h2
h2 :: Predicate
*Main> :t h3
h3 :: TV
*Main> :t h4
h4 :: SchemaLink
```

It's able to acheive something but it's very verbose. The atom
hierarchy is captured in the atoms constructors, like AConcept to tell
an Atom can be a Concept, etc. Could perhaps work, but it's really not
elegant.

The hierarchy is rather limited as well, for instance I can't define a
SchemaLink of type (Number, Number) -> Number, for that I'd have to
define another SchemaLink constructor specially for that
signature.

## Experiment with type classes (TODO)

The idea would be to represent the type hierarchy using type classes
(a type class per atom type I guess).
