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

(define extensional-similarity-evaluation-rule
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
            (GroundedSchemaNode "scm: extensional-similarity-evaluation-formula")
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

(define (extensional-similarity-evaluation-formula AB CA CB)
    (cog-set-tv!
        AB (extensional-similarity-evaluation-side-effect-free-formula AB CA CB)))

(define (extensional-similarity-evaluation-side-effect-free-formula AB CA CB)
    (let 
        ((sCA (cog-mean CA))
         (cCA (cog-confidence CA))
         (sCB (cog-mean CB))
         (cCB (cog-confidence CB)))
        (if 
            (and (< sCA 0.5) (< sCB 0.5))
            (stv 0 0)
            (if
                (and (>= sCA 0.5) (>= sCB 0.5))
                (stv 1 1)
                (stv 0 1)))))

; Name the rule
(define extensional-similarity-evaluation-rule-name
  (DefinedSchemaNode "extensional-similarity-evaluation-rule"))
(DefineLink extensional-similarity-evaluation-rule-name
  extensional-similarity-evaluation-rule)
