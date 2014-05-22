; ==========================================================================
; Compile the functions to improve performance
; ==========================================================================

(define functions-list (list
    'pln-rule-deduction
    'pln-formula-simple-deduction
    'pln-formula-simple-deduction-side-effect-free

    'pln-rule-modus-ponens
    'pln-formula-simple-modus-ponens
    'pln-formula-simple-modus-ponens-side-effect-free
))

(for-each
	(lambda (f) (compile f #:env (current-module)))
	functions-list)
