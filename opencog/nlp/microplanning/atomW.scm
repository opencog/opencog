; loading additional dependency
(use-modules (oop goops))

; -----------------------------------------------------------------------
; <atomW> -- Custom wrapper for an atom to differentiate same atoms
;
(define-class <atomW> ()
	(at #:init-keyword #:atom #:getter get-atom)			; the original atom
	(tw #:init-keyword #:time-weight #:getter get-time-weight)	; the time weight
)

