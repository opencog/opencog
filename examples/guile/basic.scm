;
; Basic guile usage example
;

(load-extension "build/opencog/guile/libsmob" "_ZN7opencog10SchemeSmob4initEv")

; Creating an atom without an atomspace to put it in is an error.
; Try it!  An error message will be printed.
(cog-new-node 'ConceptNode "asdf")

; Create a new atomspace and declare it to be the default atomspace
; for all subsequent calls.
(cog-set-atomspace! (cog-new-atomspace))

; Create an atom and place it in the default atomspace.
(cog-new-node 'ConceptNode "asdf")

; Access this newly created atom.
(cog-node 'ConceptNode "asdf")

; Access an atom that does not exist.
(cog-node 'ConceptNode "qwerty")

