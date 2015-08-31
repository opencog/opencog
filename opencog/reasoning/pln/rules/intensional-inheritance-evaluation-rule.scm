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

(define pln-rule-intensional-inheritance-evaluation
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
            (GroundedSchemaNode "scm: pln-formula-intensional-inheritance-evaluation")
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

(define (pln-formula-intensional-inheritance-evaluation AB CA CB)
    (cog-set-tv!
        AB (pln-formula-intensional-inheritance-evaluation-side-effect-free AB CA CB)))

(define (pln-formula-intensional-inheritance-evaluation-side-effect-free AB CA CB)
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
(define pln-rule-intensional-inheritance-evaluation-name
  (Node "pln-rule-intensional-inheritance-evaluation"))
(DefineLink
  pln-rule-intensional-inheritance-evaluation-name
  pln-rule-intensional-inheritance-evaluation)
