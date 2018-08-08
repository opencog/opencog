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

(define and-evaluation-rule
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
            (GroundedSchemaNode "scm: and-evaluation-formula")
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

(define (and-evaluation-formula AB CA CB)
    (cog-set-tv!
        AB (and-evaluation-side-effect-free-formula AB CA CB)))

(define (and-evaluation-side-effect-free-formula AB CA CB)
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
(define and-evaluation-rule-name (DefinedSchemaNode "and-evaluation-rule"))
(DefineLink and-evaluation-rule-name and-evaluation-rule)
