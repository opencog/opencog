; loading additional dependency
(use-modules (oop goops))

; -----------------------------------------------------------------------
; <chunks-option> -- A container containing the different options for chunking
;
; The options are:
;    main-weight-proc: the weighting procedure for selecting the best sentence-formed atom
;    supp-weight-proc: the weighting procedure for selecting the best support atom
;    form-limit: the number of sentence formed links allowed in a sentence (chunk)
;
; The procedures must accept parameter "time", "form", "link" in this order.
;
(define-class <chunks-option> ()
	(pm #:init-keyword #:main-weight-proc #:getter get-main-weight-proc)
	(ps #:init-keyword #:supp-weight-proc #:getter get-supp-weight-proc)
	(nf #:init-keyword #:form-limit #:getter get-form-limit)
)
