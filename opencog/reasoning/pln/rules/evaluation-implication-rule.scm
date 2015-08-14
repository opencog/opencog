; =============================================================================
; Evaluation Implication Rule
;
; AndLink
;   EvaluationLink
;       A
;       B
;   ImplicationLink
;       A
;       C
; |-
; EvaluationLink
;   C
;   B
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-evaluation-implication
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C")
            (EvaluationLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (ImplicationLink
                (VariableNode "$A")
                (VariableNode "$C")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-evaluation-implication")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (EvaluationLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$C"))
                (EvaluationLink
                    (VariableNode "$C")
                    (VariableNode "$B"))))))

(define (pln-formula-evaluation-implication A B C AB AC CB)
    (cog-set-tv!
        CB
        (stv
            (simple-deduction-formula 
                (cog-stv-strength B)
                (cog-stv-strength A)
                (cog-stv-strength C)
                (cog-stv-strength AB)
                (cog-stv-strength AC))
            (min
                (cog-stv-confidence B)
                (cog-stv-confidence A)
                (cog-stv-confidence C)
                (cog-stv-confidence AC)
                (* 0.9 (cog-stv-confidence AB))))))
