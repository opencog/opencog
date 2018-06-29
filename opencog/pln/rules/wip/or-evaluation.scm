; =============================================================================
; OrEvaluationRule
;
; AndLink
;   MemberLink
;       A
;       C
;   MemberLink
;       B
;       C
; |-
; OrLink
;   A
;   B
;
; -----------------------------------------------------------------------------

(define or-evaluation-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (MemberLink
                (VariableNode "$A")
                (VariableNode "$C"))
            (MemberLink
                (VariableNode "$B")
                (VariableNode "$C")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: or-evaluation-formula")
            (ListLink
                (OrLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (MemberLink
                    (VariableNode "$A")
                    (VariableNode "$C"))
                (MemberLink
                    (VariableNode "$B")
                    (VariableNode "$C"))))))

(define (or-evaluation-formula AB CA CB)
    (cog-set-tv!
        AB (or-evaluation-side-effect-free-formula AB CA CB)))

(define (or-evaluation-side-effect-free-formula AB CA CB)
    (let 
        ((sCA (cog-stv-strength CA))
         (cCA (cog-stv-confidence CA))
         (sCB (cog-stv-strength CB))
         (cCB (cog-stv-confidence CB)))
        (if 
            (and (< sCA 0.5) (< sCB 0.5))
            (stv 0 1)
            (stv 1 1))))

; Name the rule
(define or-evaluation-rule-name (DefinedSchemaNode "or-evaluation-rule"))
(DefineLink or-evaluation-rule-name or-evaluation-rule)
