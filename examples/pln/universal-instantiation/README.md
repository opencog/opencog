Universal Instantiation
=======================

Simple example of universal instantiation, using the forward chainer,
the backward chainer and the pattern matcher. 

Forward Chainer
---------------

Load the knowledge and rule base.

```scheme
(load "kb.scm")
(load "rb.scm")
```

Let's call the forward

```scheme
(pln-fc forall)
```

Backward Chainer
----------------

Clear the atomspace and reload the knowledge base.

Load the knowledge base.

```scheme
(clear
(load "kb.scm")
(load "rb.scm")
```

Pattern Matcher
---------------
