;
; Opencog spacetime atom-types module
;
(define-module (opencog spacetime))

(use-modules (opencog))

; Load the C library that calls the classserver to load the types.
(load-extension "libspacetime-types" "spacetime_types_init")

(load "spacetime/spacetime_types.scm")
