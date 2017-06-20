;
; Opencog atom-types module
;

(use-modules (opencog))  ; needed for cog-type->int and LTDL

(define-module (opencog atom-types))
; Alternately, we could also have
; (define-module (opencog atomtypes spacetime-types))
; and so on, but I don't see the point of that, at the moment...

; Load the C libraries that call the classserver to load the types.
(load-extension "libattention-types" "attention_types_init")
(load-extension "libspacetime-types" "spacetime_types_init")

(load "attention/attention_types.scm")
(load "spacetime/spacetime_types.scm")
