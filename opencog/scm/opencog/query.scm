;
; OpenCog Pattern matcher module
;

; We need (opencog extension) because  SchemePrimitive.cc hides things
; there.
(use-modules (opencog extension))

(define-module (opencog query))

(load-extension "libquery" "opencog_query_init")

