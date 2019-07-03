;
; Opencog spacetime atom-types module
;
(define-module (opencog spacetime))

(use-modules (opencog))
(use-modules (opencog oc-config))

; Load the C library that calls the nameserver to load the types.
(load-extension (string-append opencog-ext-path-spacetime-types "libspacetime-types") "spacetime_types_init")

(load "spacetime/spacetime_types.scm")
