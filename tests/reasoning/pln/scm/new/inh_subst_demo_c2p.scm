; Test substitution of similar entities, child-to-parent.
; i.e. if A inh B, replace A -> B (in other links)

(define man (ConceptNode "man" (stv 0.8 0.8)))
(define S (ConceptNode "Socrates" (stv 0.2 0.65)))
(define air (ConceptNode "air" (stv 0.1 0.9)))
(define breathes (PredicateNode "breathes" (stv 0.47 0.82)))

(define inh_mm (InheritanceLink S man (stv 0.5 0.75)))

(define breathes_S (EvaluationLink (stv 0.7 0.8)
    breathes
    (ListLink (stv 1 0)
        S
        air)))

(define breathes_man (EvaluationLink
    breathes
    (ListLink (stv 1 0)
        man
        air)))

; (pln-bc breathes_man 200)

