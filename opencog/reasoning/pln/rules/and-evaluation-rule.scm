; =============================================================================
; AndEvaluationRule
;
; AndLink
;   MemberLink
;       C
;       A
;   MemberLink
;       C
;       B
; |-
; AndLink
;   A
;   B
;
; -----------------------------------------------------------------------------

(define pln-rule-and-evaluation
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (MemberLink
                (VariableNode "$C")
                (VariableNode "$A"))
            (MemberLink
                (VariableNode "$C")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-and-evaluation")
            (ListLink
                (AndLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (MemberLink
                    (VariableNode "$C")
                    (VariableNode "$A"))
                (MemberLink
                    (VariableNode "$C")
                    (VariableNode "$B"))))))

(define (pln-formula-and-evaluation AB CA CB)
    (cog-set-tv!
        AB (pln-formula-and-evaluation-side-effect-free AB CA CB)))

(define (pln-formula-and-evaluation-side-effect-free AB CA CB)
    (let 
        ((sCA (cog-stv-strength CA))
         (cCA (cog-stv-confidence CA))
         (sCB (cog-stv-strength CB))
         (cCB (cog-stv-confidence CB)))
        (if 
            (and (>= sCA 0.5) (>= sCB 0.5))
            (stv 1 1)
            (stv 0 1))))

; Name the rule
(define pln-rule-and-evaluation-name (Node "pln-rule-and-evaluation"))
(DefineLink pln-rule-and-evaluation-name pln-rule-and-evaluation)
