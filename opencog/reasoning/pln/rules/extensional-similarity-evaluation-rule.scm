; =============================================================================
; ExtensionalSimilarityEvaluationRule
;
; AndLink
;   MemberLink
;       C
;       A
;   MemberLink
;       C
;       B
; |-
; ExtensionalSimilarityLink
;   A
;   B
;
; -----------------------------------------------------------------------------

(define pln-rule-extensional-similarity-evaluation
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
            (GroundedSchemaNode "scm: pln-formula-extensional-similarity-evaluation")
            (ListLink
                (ExtensionalSimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (MemberLink
                    (VariableNode "$C")
                    (VariableNode "$A"))
                (MemberLink
                    (VariableNode "$C")
                    (VariableNode "$B"))))))

(define (pln-formula-extensional-similarity-evaluation AB CA CB)
    (cog-set-tv!
        AB (pln-formula-extensional-similarity-evaluation-side-effect-free AB CA CB)))

(define (pln-formula-extensional-similarity-evaluation-side-effect-free AB CA CB)
    (let 
        ((sCA (cog-stv-strength CA))
         (cCA (cog-stv-confidence CA))
         (sCB (cog-stv-strength CB))
         (cCB (cog-stv-confidence CB)))
        (if 
            (and (< sCA 0.5) (< sCB 0.5))
            (stv 0 0)
            (if
                (and (>= sCA 0.5) (>= sCB 0.5))
                (stv 1 1)
                (stv 0 1)))))
