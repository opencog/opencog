Amusing honest friend
=====================

PLN demo involving deductive and abductive reasoning. Self is looking
for an amusing honest friend and infers that Bob would be one based on
his actions and the fact that friends tend to be honest (that's the
abductive part).

Step-by-step
------------

To run the inference step by step using the pattern matcher only, load
of paste each step in you guile shell from the following file

```
amusing-friend-pm.scm
```

each step is detailed here.

Forward Chainer
---------------

To run the inference using the forward chainer, load the following in
your guile shell

```scheme
(load "amusing-friend-fc.scm")
```

Once the inference is complete (a few hours) you may query each step
as explained in the file

```
amusing-friend-fc-traces.scm
```

Inference Chain Documentation
-----------------------------

The file Amusing_Friend_PLN_Demo.html documents the inference chain 
steps of the demo. The file can be opened in a web browser. Detail 
information about the PLN rules and Atomese involved at each step can be
displayed by expanding the step panel sections.
