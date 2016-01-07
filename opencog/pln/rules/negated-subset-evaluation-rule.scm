; =============================================================================
; NegatedSubsetEvaluationRule
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
;   NotLink
;       A
;   B
;
; -----------------------------------------------------------------------------

(define negated-subset-evaluation-rule
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
            (GroundedSchemaNode "scm: negated-subset-evaluation-formula")
            (ListLink
                (SubsetLink
                    (NotLink
                        (VariableNode "$A"))
                    (VariableNode "$B"))
                (MemberLink
                    (VariableNode "$C")
                    (VariableNode "$A"))
                (MemberLink
                    (VariableNode "$C")
                    (VariableNode "$B"))))))

(define (negated-subset-evaluation-formula nAB CA CB)
    (cog-set-tv!
        nAB (negated-subset-evaluation-side-effect-free-formula nAB CA CB)))

(define (subset-negated-evaluation-side-effect-free-formula nAB CA CB)
    (let 
        ((snCA (- 1 (cog-stv-strength CA)))
         (cnCA (cog-stv-confidence CA))
         (sCB (cog-stv-strength CB))
         (cCB (cog-stv-confidence CB)))
        (if 
            (< snCA 0.5)
            (stv 0 0)
            (if
                (< sCB 0.5)
                (stv 1 0)
                (stv 1 1)))))

; Name the rule
(define negated-subset-evaluation-rule-name
  (DefinedSchemaNode "negated-subset-evaluation-rule"))
(DefineLink
  negated-subset-evaluation-rule-name
  negated-subset-evaluation-rule)
