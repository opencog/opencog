; =============================================================================
; IntensionalInheritanceEvaluationRule
;
; AndLink
;   AttractionLink
;       C
;       A
;   AttractionLink
;       C
;       B
; |-
; IntensionalInheritanceLink
;   A
;   B
;
; -----------------------------------------------------------------------------

(define intensional-inheritance-evaluation-rule
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
            (GroundedSchemaNode "scm: intensional-inheritance-evaluation-formula")
            (ListLink
                (IntensionalInheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (AttractionLink
                    (VariableNode "$C")
                    (VariableNode "$A"))
                (AttractionLink
                    (VariableNode "$C")
                    (VariableNode "$B"))))))

(define (intensional-inheritance-evaluation-formula AB CA CB)
    (cog-set-tv!
        AB (intensional-inheritance-evaluation-side-effect-free-formula AB CA CB)))

(define (intensional-inheritance-evaluation-side-effect-free-formula AB CA CB)
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
(define intensional-inheritance-evaluation-rule-name
  (DefinedSchemaNode "intensional-inheritance-evaluation-rule"))
(DefineLink
  intensional-inheritance-evaluation-rule-name
  intensional-inheritance-evaluation-rule)
