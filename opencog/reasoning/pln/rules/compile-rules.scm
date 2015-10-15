; ==========================================================================
; Compile the functions to improve performance
; ==========================================================================

(define functions-list (list
    'deduction-rule
    'simple-deduction-strength-formula
    'simple-deduction-side-effect-free-formula

    'modus-ponens-rule
    'simple-modus-ponens-formula
    'simple-modus-ponens-side-effect-free-formula
))

(for-each
	(lambda (f) (compile f #:env (current-module)))
	functions-list)
