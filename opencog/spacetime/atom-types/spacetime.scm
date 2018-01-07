;
; Opencog spacetime atom-types module
;

(use-modules (opencog))

(define-module (opencog spacetime))

; Load the C library that calls the classserver to load the types.
(load-extension "libspacetime-types" "spacetime_types_init")

(load "spacetime/spacetime_types.scm")
