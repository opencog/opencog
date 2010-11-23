; example to proof a >~ c using than if x >~ y and y >~ z then x >~ z
(define agt (PredicateNode "AboutGreaterThan"))
; axiom
(define x (VariableNode "x"))
(define y (VariableNode "y"))
(define z (VariableNode "z"))
(define antecedant (AndLink (EvaluationLink agt (ListLink x y))
                            (EvaluationLink agt (ListLink y z))))
(define conclusion (EvaluationLink agt (ListLink x z)))
(define body (ImplicationLink antecedant conclusion))
(define axiom (ForAllLink (stv 0.8 0.9) (ListLink x y z) body))
; facts
(define a (ConceptNode "a"))
(define b (ConceptNode "b"))
(define c (ConceptNode "c"))
(define a_agt_b (EvaluationLink (stv .5 0.8) agt (ListLink a b)))
(define b_agt_c (EvaluationLink (stv .5 0.7) agt (ListLink b c)))

;
; Inference
; csn stands from conclusion of step n
;
(define cs1 (CustomCrispForAllRule axiom a b c)) ; universal instantiation
(define cs2 (SimpleANDRule a_agt_b b_agt_c)) ; antecedant instance
(define target (ModusPonensRule cs1 cs2))
