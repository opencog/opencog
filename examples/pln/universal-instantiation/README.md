Universal Instantiation
=======================

Simple example of universal instantiation, using the pattern matcher,
the backward and forward chainers.

Pattern Matcher
---------------

Load the knowledge and rule base in guile.

```scheme
(load "kb.scm")
(load "rb.scm")
```

Let's call meta-bind to apply universal instantiation over the entire
atomspace.

```scheme
(meta-bind universal-full-instantiation-forall-1ary-meta-rule)
```

You should get the following results.

```scheme
$9 = (SetLink
   (InheritanceLink (stv 1 1)
      (ConceptNode "Infinity")
      (ConceptNode "Abstractverse")
   )
   (InheritanceLink (stv 1 1)
      (ConceptNode "Mathematics")
      (ConceptNode "Abstractverse")
   )
   (InheritanceLink (stv 1 1)
      (ConceptNode "Space")
      (ConceptNode "Abstractverse")
   )
   (InheritanceLink (stv 1 1)
      (ConceptNode "URE")
      (ConceptNode "Abstractverse")
   )
   (InheritanceLink (stv 1 1)
      (ConceptNode "Time")
      (ConceptNode "Abstractverse")
   )
   (InheritanceLink (stv 1 1)
      (ConceptNode "Abstractverse")
      (ConceptNode "Abstractverse")
   )
   (InheritanceLink (stv 1 1)
      (ConceptNode "PLN")
      (ConceptNode "Abstractverse")
   )
   (InheritanceLink (stv 1 1)
      (ConceptNode "Cat")
      (ConceptNode "Abstractverse")
   )
)
```

Forward Chainer
---------------

```scheme
(clear)
(load "kb.scm")
(load "rb.scm")
```

Let's call the forward chainer

```scheme
(pln-fc forall)
```

which should return the same as `meta-bind` or a slightly smaller
subset of it.

Backward Chainer
----------------

Clear the atomspace and reload the knowledge base.

Load the knowledge base.

```scheme
(clear)
(load "kb.scm")
(load "rb.scm")
```

Let's call the backward chainer and ask what inherits from the
abstractverse.

```scheme
(pln-bc (Inheritance (Variable "$X") (Concept "Abstractverse"))
        (TypedVariable (Variable "$X") (Type "ConceptNode")))
```

which should return the same as `meta-bind` or a slightly smaller
subset of it.
