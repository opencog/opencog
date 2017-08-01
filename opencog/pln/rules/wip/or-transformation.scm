; =====================================================================
; OrTransformationRule
;
; OrLink 
;    A
;    B
; |-
; (NotLink A) => B
;----------------------------------------------------------------------


(define or-transformation-rule
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B"))
   (OrLink
    (VariableNode "$A")
    (VariableNode "$B"))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: or-transformation-formula")
    (ListLink
     (OrLink
      (VariableNode "$A")
      (VariableNode "$B"))
     (ImplicationLink
      (NotLink 
       (VariableNode "$A"))
      (VariableNode "$B"))))))

(define (or-transformation-formula OAB IAB)
  (cog-set-tv!
   IAB
   (or-transformation-side-effect-free-formula OAB IAB))
)


(define (or-transformation-side-effect-free-formula OAB IAB)
  (let 
      ((sOAB (cog-stv-strength OAB))
       (cOAB (cog-stv-confidence OAB)))
    (stv sOAB cOAB)))

; Name the rule
(define or-transformation-rule-name (DefinedSchemaNode "or-transformation-rule"))
(DefineLink or-transformation-rule-name or-transformation-rule)
