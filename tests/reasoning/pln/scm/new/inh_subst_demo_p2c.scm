; Test substitution of similar entities, parent-to-child.
; i.e. if A inh B, replace B -> A (in other links)
; This file is the same as the c2p version, but with the TV moved.

(define man (ConceptNode "man" (stv 0.8 0.8)))
(define S (ConceptNode "Socrates" (stv 0.2 0.65)))
(define air (ConceptNode "air" (stv 0.1 0.9)))
(define breathes (PredicateNode "breathes" (stv 0.47 0.82)))

(define inh_mm (InheritanceLink S man (stv 0.5 0.75)))

(define breathes_S (EvaluationLink
    breathes
    (ListLink (stv 1 0)
        S
        air)))

(define breathes_man (EvaluationLink (stv 0.7 0.8)
    breathes
    (ListLink (stv 1 0)
        man
        air)))

; (pln-bc breathes_S 200)
