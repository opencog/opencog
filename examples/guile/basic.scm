;
; Basic guile usage example
; Unfinished ...
;
; See opencog/guile/README or http://wiki.opencog.org/w/Scheme
; for additional documentation.
;
; export LTDL_LIBRARY_PATH=build/opencog/guile
; guile -L opencog/scm
; or add it to your ~/.guile file
;
; (%search-load-path "build/opencog/scm")

(use-modules (opencog))

; Create an atom and place it in the default atomspace.
(cog-new-node 'ConceptNode "asdf")

; Access this newly created atom.
(cog-node 'ConceptNode "asdf")

; Access an atom that does not exist.
(cog-node 'ConceptNode "qwerty")

; Using opencog is easier if we load some additional modules
(use-modules (opencog atomtypes))

; XXX this won't work until we load a bunch of scm files...
(ConceptNode "foobar")
