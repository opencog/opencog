
ECAN and the Attention Bank
===========================
ECAN is the Economic Attention Allocation System.  The general idea is
that the attention-span is limited, and that most computation happens
near where the attention is distributed. Because the supply of money is
limited, it is treated like "money", that is, economically. The bank
is the subystem that keeps track of how much of it there is.


Scheme bindings
---------------
These are the scheme bindings.

### cog-new-av sti lti vlti

Create an AttentionValue with the given STI, LTI and VLTI.
Unlike atoms, attention values are ephemeral: they are automatically
garbage-collected when no longer needed.

Example:

    ; Create a new attention value:
    guile> (cog-new-av 10 20 0)


### cog-av? exp

Return #t if exp is an attention value, else return #f

Example:

    ; Define a simple attention value
    guile> (define x (cog-new-av 15 25 0))
    guile> (define y (+ 2 2))
    guile> (cog-av? x)
    #t
    guile> (cog-av? y)
    #f

### cog-av atom

Return the attention value of the atom.

Example:

    ; Define a node
    guile> (define x
              (cog-new-node 'ConceptNode "abc"
                 (cog-new-av 11 21 0)))
    guile> (cog-av x)
    (av 11 21 0)
    guile> (cog-av? (cog-av x))
    #t

### cog-set-av! atom av
Set the attention value of the atom.

Example:

    ; Define a node
    guile> (define x (cog-new-node 'ConceptNode "def"))
    guile> (cog-av x)
    (av 0 0 0)
    guile> (cog-set-av! x (cog-new-av 44 55 1))
    (ConceptNode "def" (av 44 55 1))
    guile> (cog-av x)
    (av 44 55 1)

### cog-inc-vlti! atom av
Increase the vlti of the atom by 1.

Example:

    ; Define a node
    guile> (define x
              (cog-new-node 'ConceptNode "abc"
                 (cog-new-av 11 21 0)))
    guile> (cog-inc-vlti! x)
    (ConceptNode "abc" (av 11 21 1))
    guile> (cog-av x)
    (av 11 21 1)
    guile> (cog-inc-vlti! x)
    (ConceptNode "abc" (av 11 21 2))
    guile> (cog-av x)
    (av 11 21 2)

### cog-dec-vlti! atom av
Decrease the vlti of the atom by 1.

Example:

    ; Define a node
    guile> (define x
              (cog-new-node 'ConceptNode "abc"
                 (cog-new-av 11 21 1)))
    guile> (cog-dec-vlti! x)
    (ConceptNode "abc" (av 11 21 0))
    guile> (cog-av x)
    (av 11 21 0)

### cog-av->alist av
Convert an attention value to an association list (alist).

Example:

    guile> (define x (cog-new-av 99 88 0))
    guile> (cog-av->alist x)
    ((sti . 99) (lti . 88) (vlti . 0))
