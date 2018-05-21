;
; Opencog attention module
;

(use-modules (opencog))  ; needed for cog-type->int and LTDL

(define-module (opencog attention))

; Load the C library that calls the nameserver to load the types.
(load-extension "libattention-types" "attention_types_init")

(load "attention/attention_types.scm")
(load "attention/default-param-values.scm")
