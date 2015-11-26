(define-module (opencog nlp microplanning))

(use-modules (srfi srfi-1)
             (opencog)
             (opencog nlp)  ; need the atom types
             (opencog nlp relex2logic) ; helpers.scm uses this
             (opencog nlp sureal))

;
; loading additional dependency
(load-from-path "microplanning/main.scm")
