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

(define pln-rule-negated-subset-evaluation
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
            (GroundedSchemaNode "scm: pln-formula-negated-subset-evaluation")
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

(define (pln-formula-negated-subset-evaluation nAB CA CB)
    (cog-set-tv!
        nAB (pln-formula-negated-subset-evaluation-side-effect-free nAB CA CB)))

(define (pln-formula-subset-negated-evaluation-side-effect-free nAB CA CB)
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
(define pln-rule-negated-subset-evaluation-name
  (Node "pln-rule-negated-subset-evaluation"))
(DefineLink
  pln-rule-negated-subset-evaluation-name
  pln-rule-negated-subset-evaluation)
