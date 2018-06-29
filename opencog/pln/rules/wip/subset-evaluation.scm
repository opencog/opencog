; =============================================================================
; SubsetEvaluationRule
;
; AndLink
;   MemberLink
;       A
;       C
;   MemberLink
;       B
;       C
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
                (VariableNode "$A")
                (VariableNode "$C"))
            (MemberLink
                (VariableNode "$B")
                (VariableNode "$C")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: subset-evaluation-formula")
            (ListLink
                (SubsetLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (MemberLink
                    (VariableNode "$A")
                    (VariableNode "$C"))
                (MemberLink
                    (VariableNode "$B")
                    (VariableNode "$C"))))))

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
