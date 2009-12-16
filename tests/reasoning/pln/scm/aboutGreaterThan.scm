; example to proof a ~> c using than if x ~> y and y ~> z then x ~> z
; axiom
(define agt (PredicateNode "AboutGreaterThan"))
(define x (VariableNode "x"))
(define y (VariableNode "y"))
(define z (VariableNode "z"))
(ImplicationLink (stv 0.8 0.9)
    (AndLink
        (EvaluationLink
            agt
            (ListLink x y))
        (EvaluationLink
            agt
            (ListLink y z))
        )
    (EvaluationLink
        agt
        (ListLink x z)
        )
    )
; facts
(define a (ConceptNode "a"))
(define b (ConceptNode "b"))
(define c (ConceptNode "c"))
(define a_agt_b (EvaluationLink (stv .5 0.8) agt (ListLink a b)))
(define b_agt_c (EvaluationLink (stv .5 0.7) agt (ListLink b c)))
(define a_agt_c (EvaluationLink agt (ListLink a c)))
; intermediate rule (for start)
(define impl
  (ImplicationLink (stv 0.8 0.9)
      (AndLink a_agt_b b_agt_c)
      a_agt_c))
;
; Inference
; csn stands from conclusion of step n
;
(define cs1 (SimpleANDRule a_agt_b b_agt_c))
(define target (ModusPonensRule impl cs1))
