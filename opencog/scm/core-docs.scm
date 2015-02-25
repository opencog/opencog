(set-procedure-property! cog-new-node 'documentation
"
 cog-new-node node-type node-name
    Create a new node of the given type and name

    Optionally, a truth value and/or an attention value can follow
    the node name.

    Throws errors if node-type is not a valid atom type for a node,
    and if node-name is not a string.

    Example:
        ; Create a new node, and prints its value:
        guile> (cog-new-node 'ConceptNode \"some node name\")
        (ConceptNode \"some node name\")

        ; Creates a new node, with a truth value:
        guile> (cog-new-node 'ConceptNode \"another node\"
                      (cog-new-stv 0.8 0.9))
        (ConceptNode \"another node\" (stv 0.8 0.9))
")

(set-procedure-property! cog-node 'documentation
"
 cog-node node-type node-name
    Returns the node of the given type and name, if it exists, else
    returns null.

    Optionally, a truth value and/or attention value can follow the
    node name. If the node exists, then the truth value and/or
    attention value is modified.

    Throws errors if node-type is not a valid atom type for a node,
    and if node-name is not a string.

    Example:
        ; Check to see if a node exists:
        guile> (cog-node 'ConceptNode \"asdf\")
        ()

        ; Verify that the return value is actually a true null:
        guile> (null? (cog-node 'ConceptNode \"asdf\"))
        #t

        ; Now, create the node, and see if it exists:
        guile> (cog-new-node 'ConceptNode \"asdf\")
        (ConceptNode \"asdf\")
        guile> (null? (cog-node 'ConceptNode \"asdf\"))
        #f

        ; Change the truth value of an existing node:
        guile> (cog-node 'ConceptNode \"asdf\" (cog-new-stv 0.8 0.9))
        (ConceptNode \"asdf\" (stv 0.8 0.9))
")

(set-procedure-property! cog-new-link 'documentation
"
 cog-new-link link-type atom ... atom
    Create a new link, with the given atoms in the link.

    Optionally, a truth value or an attention value can be included
    in the list of atoms.

    Throws errors if the link type is not a valid opencog link type,
    or if any of the arguments after the link type are not atoms,
    truth values, attention values, or nil.

    Example:
        ; Creates two nodes, and a new link:
        guile> (define x (cog-new-node 'ConceptNode \"abc\"))
        guile> (define y (cog-new-node 'ConceptNode \"def\"))
        guile> (cog-new-link 'Link x y)
        (Link
           (ConceptNode \"abc\")
           (ConceptNode \"def\")
        )

        ; Create a new link with a truth value:
        guile> (cog-new-link 'Link x y (cog-new-stv 0.7 0.8))
        (Link (stv 0.7 0.8)
           (ConceptNode \"abc\")
           (ConceptNode \"def\")
        )
")

(set-procedure-property! cog-link 'documentation
"
 cog-link link-type atom ... atom
    Returns the link of the given type and list of atoms, if it
    exists, else returns null.

    Optionally, a truth value or attention value can be included
    in the list of atoms. If the link exists, then the truth value
    and/or attention value is modified.

    Throws errors if the link type is not a valid opencog link type,
    or if any of the arguments after the link type are not atoms,
    truth values, attention values, or nil.

    Example:
        ; Create two nodes:
        guile> (define x (cog-new-node 'ConceptNode \"abc\"))
        guile> (define y (cog-new-node 'ConceptNode \"def\"))

        ; Does a node with these two links exist?
        guile> (cog-link 'Link x y)
        ()

        ; Now, create such a link
        guile> (cog-new-link 'Link x y)
        (Link
           (ConceptNode \"abc\")
           (ConceptNode \"def\")
        )

        ; Check again for existence:
        guile> (cog-link 'Link x y)
        (Link
           (ConceptNode \"abc\")
           (ConceptNode \"def\")
        )

        ; Change the truth value of an existing node:
        guile> (cog-link 'Link x y (cog-new-stv 0.7 0.8))
        (Link (stv 0.7 0.8)
           (ConceptNode \"abc\")
           (ConceptNode \"def\")
        )
")

(set-procedure-property! cog-delete 'documentation
"
 cog-delete atom
    Delete the indicated atom, but only if it has no incoming links.

    Returns #t if the atom was deleted, else returns #f if not deleted.\"
")

(set-procedure-property! cog-delete-recursive 'documentation
"
 cog-delete-recursive atom
    Delete the indicated atom, and all atoms that point at it.

    Both functions return #t on success, else they return #f.
    If #f is returned, then the delete failed.

    Example:
       ; Define two nodes and a link between them:
       guile> (define x (cog-new-node 'ConceptNode \"abc\"))
       guile> (define y (cog-new-node 'ConceptNode \"def\"))
       guile> (define l (cog-new-link 'Link x y))

       ; Verify that there's an atom called x:
       guile> x
       (Link
          (ConceptNode \"abc\")
          (ConceptNode \"def\")
       )

       ; Try to delete x. This should fail, since there's a link
       ; containing x.
       guile> (cog-delete x)
       #f

       ; Delete x, and everything pointing to it. This should delete
       ; both x, and the link l.
       guile> (cog-delete-recursive x)
       #t

       ; Verify that the link l is gone:
       guile> l
       Invalid handle

       ; Verify that the node x is gone:
       guile> x
       Invalid handle

       ; Verify that the node y still exists:
       guile> y
       (ConceptNode \"def\")
")

(set-procedure-property! cog-atom? 'documentation
"
 cog-atom? exp
    Return #t if exp is an atom, else return #f

    Example:
       ; Define a node
       guile> (define x (cog-new-node 'ConceptNode \"abc\"))
       guile> (define y (+ 2 2))
       guile> (cog-atom? x)
       #t
       guile> (cog-atom? y)
       #f
")

(set-procedure-property! cog-node? 'documentation
"
 cog-node? exp
    Return #t if exp is an node, else return #f

    See also cog-node, which will check to see if a specific node
    already exists.

    Example:
       ; Define a node and a link
       guile> (define x (cog-new-node 'ConceptNode \"abc\"))
       guile> (define y (cog-new-link 'ListLink x))
       guile> (cog-node? x)
       #t
       guile> (cog-node? y)
       #f
")

(set-procedure-property! cog-link? 'documentation
"
 cog-link? exp
    Return #t if exp is an link, else return #f

    See also cog-link, which will check to see if a specific link
    already exists.

    Example:
       ; Define a node and a link
       guile> (define x (cog-new-node 'ConceptNode \"abc\"))
       guile> (define y (cog-new-link 'ListLink x))
       guile> (cog-link? x)
       #f
       guile> (cog-link? y)
       #t
")

(set-procedure-property! cog-name 'documentation
"
 cog-name atom
    Return the name of the node. If the atom is not a node,
    returns NIL.

    Example:
       ; Define a node
       guile> (define x (cog-new-node 'ConceptNode \"abc\"))
       guile> (cog-name x)
       \"abc\"
")

(set-procedure-property! cog-type 'documentation
"
 cog-type atom
    Return the type of the atom.

    Example:
       ; Define a node
       guile> (define x (cog-new-node 'ConceptNode \"abc\"))
       guile> (cog-type x)
       ConceptNode
       guile> (eq? 'ConceptNode (cog-type x))
       #t
")

(set-procedure-property! cog-arity 'documentation
"
 cog-arity atom
    Return the arity of the atom.

    Example:
       guile> (define x (cog-new-node 'ConceptNode \"abc\"))
       guile> (cog-arity x)
       0
       guile> (define l (cog-new-link 'Link x x x))
       guile> (cog-arity l)
       3
")

(set-procedure-property! cog-incoming-set 'documentation
"
 cog-incoming-set atom
    Return the incoming set of the atom.  This set is returned as an
    ordinary scheme list.

    Example:
       ; Define two nodes and a link between them:
       guile> (define x (cog-new-node 'ConceptNode \"abc\"))
       guile> (define y (cog-new-node 'ConceptNode \"def\"))
       guile> (define l (cog-new-link 'Link x y))

       ; Get the incoming sets of nodes x and y (which is the link l):
       guile> (cog-incoming-set x)
       ((Link
          (ConceptNode \"abc\")
          (ConceptNode \"def\")
       )
       )

       guile> (cog-incoming-set y)
       ((Link
          (ConceptNode \"abc\")
          (ConceptNode \"def\")
       )
       )

       ; Verify that the both incoming sets are really one and the
       ; same link:
       guile> (equal? (cog-incoming-set x) (cog-incoming-set y))
       #t

       ; The returned values are lists, and not singleton atoms.
       ; Thus, the incoming set of x is a list containing l:
       guile> (equal? (cog-incoming-set x) (list l))
       #t

       ; Verify that the returned value is a true list:
       guile> (list? (cog-incoming-set x))
       #t
")

(set-procedure-property! cog-outgoing-set 'documentation
"
 cog-outgoing-set atom
    Return the outgoing set of the atom.  This set is returned as an
    ordinary scheme list.
")

(set-procedure-property! cog-atom 'documentation
"
 cog-atom handle
    Reference the atom identified by the integer-valued handle
")

(set-procedure-property! cog-handle 'documentation
"
 cog-handle atom
    Return the handle (which is an integer) of the atom

    It may be useful to remember that scheme indicates hexadecimal
    numbers by preceeding them with #x, and so, for example,
    (cog-atom #x2c949b) gets the handle associated with hex 2c949b.

    Example:
       ; Create two atoms, and get thier handles:
       guile> (define x (cog-new-node 'ConceptNode \"abc\"))
       guile> (define y (cog-new-node 'ConceptNode \"def\"))
       guile> (cog-handle x)
       113
       guile> (cog-handle y)
       114

       ; Get the atom corresponding to handle number 114
       guile> (cog-atom 114)
       (ConceptNode \"abc\")

       ; Verify that handles are truly integers
       guile> (integer? x)
       #f
       guile> (integer? (cog-handle x))
       #t
")

(set-procedure-property! cog-new-stv 'documentation
"
 cog-new-stv mean confidence
    Create a SimpleTruthValue with the given mean and confidence.
    Unlike atoms, truth values are ephemeral: they are automatically
    garbage-collected when no longer needed.

    Throws errors if mean and confidence are not floating-point
    values.
    Example:
        ; Create a new simple truth value:
        guile> (cog-new-stv 0.7 0.9)
")

(set-procedure-property! cog-new-ctv 'documentation
"
 cog-new-ctv mean confidence count
    Create a CountTruthValue with the given mean, confidence and count.
    Unlike atoms, truth values are ephemeral: they are automatically
    garbage-collected when no longer needed.

    Throws errors if mean, confidence and count are not floating-point
    values.
    Example:
        ; Create a new count truth value:
        guile> (cog-new-ctv 0.7 0.9 44.0)
")

(set-procedure-property! cog-new-itv 'documentation
"
 cog-new-itv lower upper confidence
    Create an IndefiniteTruthValue with the given lower, upper and confidence.
    Unlike atoms, truth values are ephemeral: they are automatically
    garbage-collected when no longer needed.

    Throws errors if lower, upper and confidence are not floating-point
    values.
    Example:
        ; Create a new indefinite truth value:
        guile> (cog-new-itv 0.7 0.9 0.6)
")

(set-procedure-property! cog-new-ptv 'documentation
"
 cog-new-ptv mean confidence count
    Create a ProbabilisticTruthValue with the given mean, confidence and count.
    Unlike atoms, truth values are ephemeral: they are automatically
    garbage-collected when no longer needed.

    Throws errors if mean, confidence and count are not floating-point
    values.
    Example:
        ; Create a new probabilistic truth value:
        guile> (cog-new-ptv 0.7 0.9 44.0)
")

(set-procedure-property! cog-new-ftv 'documentation
"
 cog-new-ftv mean confidence
    Create a FuzzyTruthValue with the given mean and confidence.
    Unlike atoms, truth values are ephemeral: they are automatically
    garbage-collected when no longer needed.

    Throws errors if mean and confidence are not floating-point
    values.
    Example:
        ; Create a new fuzzy truth value:
        guile> (cog-new-ftv 0.7 0.9)
")

(set-procedure-property! cog-tv? 'documentation
"
 cog-tv? exp
    Return #t if exp is a truth value, else return #f

    Example:
       ; Define a simple truth value
       guile> (define x (cog-new-stv 0.7 0.9))
       guile> (define y (+ 2 2))
       guile> (cog-tv? x)
       #t
       guile> (cog-tv? y)
       #f
")

(set-procedure-property! cog-stv? 'documentation
"
 cog-stv? exp
    Return #t if exp is a SimpleTruthValue, else return #f
")

(set-procedure-property! cog-ctv? 'documentation
"
 cog-ctv? exp
    Return #t if exp is a CountTruthValue, else return #f
")

(set-procedure-property! cog-itv? 'documentation
"
 cog-itv? exp
    Return #t if exp is a IndefiniteTruthValue, else return #f
")

(set-procedure-property! cog-ptv? 'documentation
"
 cog-ptv? exp
    Return #t if exp is a ProbablisticTruthValue, else return #f
")

(set-procedure-property! cog-ftv? 'documentation
"
 cog-ftv? exp
    Return #t if exp is a FuzzyTruthValue, else return #f
")

(set-procedure-property! cog-tv 'documentation
"
 cog-tv atom
    Return the truth-value of the atom.

    Example:
       ; Define a node
       guile> (define x
                 (cog-new-node 'ConceptNode \"abc\"
                    (cog-new-stv 0.2 0.5)))
       guile> (cog-tv x)
       (stv 0.2 0.5)
       guile> (cog-tv? (cog-tv x))
       #t
")

(set-procedure-property! cog-set-tv! 'documentation
"
 cog-set-tv! atom tv
    Set the truth-value of the atom.

    Example:
       ; Define a node
       guile> (define x (cog-new-node 'ConceptNode \"def\"))
       guile> (cog-tv x)
       (stv 0 0)
       guile> (cog-set-tv! x (cog-new-stv 0.9 0.8))
       (ConceptNode \"def\" (stv 0.9 0.8))
       guile> (cog-tv x)
       (stv 0.9 0.8)
")

(set-procedure-property! cog-tv->alist 'documentation
"
 cog-tv->alist tv
    Convert a truth value to an association list (alist).

    Example:
       guile> (define x (cog-new-stv 0.7 0.9))
       guile> (cog-tv->alist x)
       ((mean . 0.7) (confidence . 0.9))
")

(set-procedure-property! cog-new-av 'documentation
"
 cog-new-av sti lti vlti
    Create an AttentionValue with the given STI, LTI and VLTI.
    Unlike atoms, attention values are ephemeral: they are automatically
    garbage-collected when no longer needed.

    Example:
        ; Create a new attention value:
        guile> (cog-new-av 10 20 0)
")

(set-procedure-property! cog-av? 'documentation
"
 cog-av? exp
    Return #t if exp is an attention value, else return #f

    Example:
       ; Define a simple attention value
       guile> (define x (cog-new-av 15 25 0))
       guile> (define y (+ 2 2))
       guile> (cog-av? x)
       #t
       guile> (cog-av? y)
       #f
")

(set-procedure-property! cog-av 'documentation
"
 cog-av atom
    Return the attention value of the atom.

    Example:
       ; Define a node
       guile> (define x
                 (cog-new-node 'ConceptNode \"abc\"
                    (cog-new-av 11 21 0)))
       guile> (cog-av x)
       (av 11 21 0)
       guile> (cog-av? (cog-av x))
       #t
")

(set-procedure-property! cog-set-av! 'documentation
"
 cog-set-av! atom av
    Set the attention value of the atom.

    Example:
       ; Define a node
       guile> (define x (cog-new-node 'ConceptNode \"def\"))
       guile> (cog-av x)
       (av 0 0 0)
       guile> (cog-set-av! x (cog-new-av 44 55 1))
       (ConceptNode \"def\" (av 44 55 1))
       guile> (cog-av x)
       (av 44 55 1)
")

(set-procedure-property! cog-inc-vlti! 'documentation
"
 cog-inc-vlti! atom av
    Increase the vlti of the atom by 1.

    Example:
       ; Define a node
       guile> (define x
                 (cog-new-node 'ConceptNode \"abc\"
                    (cog-new-av 11 21 0)))
       guile> (cog-inc-vlti! x)
       (ConceptNode \"abc\" (av 11 21 1))
       guile> (cog-av x)
       (av 11 21 1)
       guile> (cog-inc-vlti! x)
       (ConceptNode \"abc\" (av 11 21 2))
       guile> (cog-av x)
       (av 11 21 2)
")

(set-procedure-property! cog-dec-vlti! 'documentation
"
 cog-dec-vlti! atom av
    Decrease the vlti of the atom by 1.

    Example:
       ; Define a node
       guile> (define x
                 (cog-new-node 'ConceptNode \"abc\"
                    (cog-new-av 11 21 1)))
       guile> (cog-dec-vlti! x)
       (ConceptNode \"abc\" (av 11 21 0))
       guile> (cog-av x)
       (av 11 21 0)
")

(set-procedure-property! cog-av->alist 'documentation
"
 cog-av->alist av
    Convert an attention value to an association list (alist).

    Example:
       guile> (define x (cog-new-av 99 88 0))
       guile> (cog-av->alist x)
       ((sti . 99) (lti . 88) (vlti . 0))
")

(set-procedure-property! cog-af-boundary 'documentation
"
 cog-af-boundary
    Return the AttentionalFocus Boundary of the AtomSpace (which is
    a short integer STI value).

    Example:

    guile> (cog-af-boundary)
    100
")

(set-procedure-property! cog-set-af-boundary! 'documentation
"
 cog-set-af-boundary! int
    Set the AttentionalFocus Boundary of the AtomSpace (which is a
    short integer STI value). Returns the new AttentionalFocus boundary
    (which is a short integer STI value).

    Example:
    guile> (cog-set-af-boundary! 200)
    200
")

(set-procedure-property! cog-af 'documentation
"
 cog-af
    Return the list of atoms in the AttentionalFocus.

    Example:
    guile> (cog-af)
    (ConceptNode \"ArtificialIntelligence\" (av 15752 0 0))
    (ConceptNode \"Databases\" (av 15752 0 0))
")

(set-procedure-property! cog-get-types 'documentation
"
 cog-get-types
    Return a list of all of the atom types in the system.

    Example:
        guile> (cog-get-types)
")

(set-procedure-property! cog-type? 'documentation
"
 cog-type? symbol
    Return #t if the symbol names an atom type, else return #f

    Example:
        guile> (cog-type? 'ConceptNode)
        #t
        guile> (cog-type? 'FlorgleBarf)
        #f
")

(set-procedure-property! cog-node-type? 'documentation
"
 cog-node-type? symbol
    Return #t if the symbol names an node type, else return #f

    Example:
        guile> (cog-node-type? 'ConceptNode)
        #t
        guile> (cog-node-type? 'ListLink)
        #f
        guile> (cog-node-type? 'FlorgleBarf)
        #f
")

(set-procedure-property! cog-link-type? 'documentation
"
 cog-link-type? symbol
    Return #t if the symbol names a link type, else return #f

    Example:
        guile> (cog-link-type? 'ConceptNode)
        #f
        guile> (cog-link-type? 'ListLink)
        #t
        guile> (cog-link-type? 'FlorgleBarf)
        #f
")

(set-procedure-property! cog-type->int 'documentation
"
 cog-type->int type
    Return the integer value corresponding to an atom type.

    Example:
        guile> (cog-type->int 'ListLink)
        8
")

(set-procedure-property! cog-get-subtypes 'documentation
"
 cog-get-subtypes type
    Return a list of the subtypes of the given type.  Only the
    immediate subtypes are returned; to obtain all subtypes, this
    function should be called recursively.

    Example:
        guile> (cog-get-subtypes 'Atom)
        (Link Node)
")

(set-procedure-property! cog-subtype? 'documentation
"
 cog-subtype? type subtype
    Return #t if 'subtype' is a subtype of 'type', else return #f.
    The check is performed recursively.

    Example:
        guile> (cog-subtype? 'Node 'Link)
        #f
        guile> (cog-subtype? 'Atom 'Link)
        #t
        guile> (cog-subtype? 'Atom 'ConceptNode)
        #t
")

(set-procedure-property! cog-map-type 'documentation
"
 cog-map-type proc type
    Call proceedure proc for each atom in the atomspace that is of
    type type. If proc returns any value other than #f, then the
    iteration is terminated.  Note that this iterates only over the
    given type, and not its sub-types. Thus (cog-map-type proc 'Atom)
    will never call proc, because no atoms in the atomspace can have
    the type Atom: they are all subtypes of Atom.

    Example:
       ; define a function that prints the atoms:
       guile> (define (prt-atom h) (display h) #f)
       guile> (cog-map-type prt-atom 'ConceptNode)
")

(set-procedure-property! cog-bind 'documentation
"
 cog-bind handle
     Run pattern matcher on handle.  handle must be a BindLink.
")

(set-procedure-property! cog-bind-crisp 'documentation
"
 cog-bind-crisp handle
    Run pattern matcher on handle.  handle must be a BindLink.
    Use crisp logic during implication.  This allows NotLink clauses
    to be used to exclude certain patterns.
")

(set-procedure-property! cog-bind-single 'documentation
"
 cog-bind-single handle
    Run pattern matcher on handle.  handle must be a BindLink.
    The search is terminated after the first match is found.
")

(set-procedure-property! cog-bind-pln 'documentation
"
 cog-bind-pln handle
    Run pattern matcher on handle.  handle must be a BindLink.
    A special-purpose pattern matcher used by PLN.
")

(set-procedure-property! cog-validate-bindlink 'documentation
"
 cog-validate-bindlink handle
    Validate that the indicated handle is a BindLink constructed
    with the appropriate syntax.  This will throw an error if the
    handle is not a syntactically correct BindLink.
")

(set-procedure-property! fetch-atom 'documentation
"
 fetch-atom handle
    Fetch indicated atom from SQL/persistent storage.
")

(set-procedure-property! fetch-incoming-set 'documentation
"
 fetch-incoming-set
    Fetch the incoming set of the atom from SQL storage. The fetch is
    recursive.
")

(set-procedure-property! store-atom 'documentation
"
 store-atom handle
    Store indicated atom to SQL/persistent storage.
")

(set-procedure-property! load-atoms-of-type 'documentation
"
 load-atoms-of-type type
    Fetch atoms of the given type from SQL/persistent storage.
")

(set-procedure-property! barrier 'documentation
"
 barrier
    Block until the SQL Atom write queues are empty.
")

;(set-procedure-property! cog-yield 'documentation
;"
; cog-yield
;    The implementation uses a simple exception mechanism to allow
;    scheme code to return to the opencog server from anywhere. To use
;    this, simply throw 'cog-yield from anywhere.  The catch handler
;    will promptly return to the cogserver.  This can be used with
;    continuations to implement some simple multi-threading.
;
;    Example:
;       guile> (throw 'cog-yield \"hello world\")
;       (hello world)
;")
