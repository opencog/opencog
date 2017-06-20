;
; Opencog atom-types module
;

(use-modules (opencog) (opencog nlp) (opencog attention))

(define-module (opencog atom-types))

; Load the C library that calls the classserver to load the types.
(load-extension "libspacetime-types" "spacetime_types_init")

(load "spacetime/spacetime_types.scm")
