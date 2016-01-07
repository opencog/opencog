; =============================================================================
; SubsetEvaluationRule
;
; AndLink
;   MemberLink
;       C
;       A
;   MemberLink
;       C
;       B
; |-
; SubsetLink
;   A
;   B
;
; -----------------------------------------------------------------------------

(define subset-evaluation-rule
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
            (GroundedSchemaNode "scm: subset-evaluation-formula")
            (ListLink
                (SubsetLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (MemberLink
                    (VariableNode "$C")
                    (VariableNode "$A"))
                (MemberLink
                    (VariableNode "$C")
                    (VariableNode "$B"))))))

(define (subset-evaluation-formula AB CA CB)
    (cog-set-tv!
        AB (subset-evaluation-side-effect-free-formula AB CA CB)))

(define (subset-evaluation-side-effect-free-formula AB CA CB)
    (let 
        ((sCA (cog-stv-strength CA))
         (cCA (cog-stv-confidence CA))
         (sCB (cog-stv-strength CB))
         (cCB (cog-stv-confidence CB)))
        (if 
            (< sCA 0.5)
            (stv 0 0)
            (if
                (< sCB 0.5)
                (stv 1 0)
                (stv 1 1)))))

; Name the rule
(define subset-evaluation-rule-name (DefinedSchemaNode "subset-evaluation-rule"))
(DefineLink subset-evaluation-rule-name subset-evaluation-rule)
