; =====================================================================
; OrTransformationRule
;
; OrLink 
;    A
;    B
; |-
; (NotLink A) => B
;----------------------------------------------------------------------


(define pln-rule-or-transformation
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B"))
   (OrLink
    (VariableNode "$A")
    (VariableNode "$B"))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: pln-formula-or-transformation")
    (ListLink
     (OrLink
      (VariableNode "$A")
      (VariableNode "$B"))
     (ImplicationLink
      (NotLink 
       (VariableNode "$A"))
      (VariableNode "$B"))))))

(define (pln-formula-or-transformation OAB IAB)
  (cog-set-tv!
   IAB
   (pln-formula-or-transformation-side-effect-free OAB IAB))
)


(define (pln-formula-or-transformation-side-effect-free OAB IAB)
  (let 
      ((sOAB (cog-stv-strength OAB))
       (cOAB (cog-stv-confidence OAB)))
    (stv sOAB cOAB)))

; Name the rule
(define pln-rule-or-transformation-name (Node "pln-rule-or-transformation"))
(DefineLink pln-rule-or-transformation-name pln-rule-or-transformation)
