; =============================================================================
; IntensionalSimilarityEvaluationRule
;
; AndLink
;   AttractionLink
;       C
;       A
;   AttractionLink
;       C
;       B
; |-
; IntensionalSimilarityLink
;   A
;   B
;
; -----------------------------------------------------------------------------

(define intensional-similarity-evaluation-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (AttractionLink
                (VariableNode "$C")
                (VariableNode "$A"))
            (AttractionLink
                (VariableNode "$C")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: intensional-similarity-evaluation-formula")
            (ListLink
                (IntensionalSimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (AttractionLink
                    (VariableNode "$C")
                    (VariableNode "$A"))
                (AttractionLink
                    (VariableNode "$C")
                    (VariableNode "$B"))))))

(define (intensional-similarity-evaluation-formula AB CA CB)
    (cog-set-tv!
        AB (intensional-similarity-evaluation-side-effect-free-formula AB CA CB)))

(define (intensional-similarity-evaluation-side-effect-free-formula AB CA CB)
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

; Name the rule
(define intensional-similarity-evaluation-rule-name
  (DefinedSchemaNode "intensional-similarity-evaluation-rule"))
(DefineLink
  intensional-similarity-evaluation-rule-name
  intensional-similarity-evaluation-rule)
