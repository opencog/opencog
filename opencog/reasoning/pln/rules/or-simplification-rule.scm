; =====================================================================
; OrSimplificationRule
;
; OrLink 
;    A
;    OrLink
;       B
;	C
; |-
; OrLink
;    A
;    B
;    C
;----------------------------------------------------------------------


(define pln-rule-or-simplification
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B")
    (VariableNode "$C"))
   (OrLink
     (VariableNode "$A")
     (OrLink
      (VariableNode "$B")
      (VariableNode "$C")))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: pln-formula-or-simplification")
    (ListLink
     (OrLink
      (VariableNode "$A")
      (OrLink
       (VariableNode "$B")
       (VariableNode "$C")))
     (OrLink
      (VariableNode "$A")
      (VariableNode "$B")
      (VariableNode "$C"))))))

(define (pln-formula-or-simplification AABC ABC)
  (cog-set-tv!
   ABC
   (pln-formula-or-simplification-side-effect-free AABC ABC))
)

(define (pln-formula-or-simplification-side-effect-free AABC ABC)
  (let 
      ((sAABC (cog-stv-strength AABC))
       (cAABC (cog-stv-confidence AABC)))
    (stv sAABC cAABC)))

; Name the rule
(define pln-rule-or-simplification-name (Node "pln-rule-or-simplification"))
(DefineLink pln-rule-or-simplification-name pln-rule-or-simplification)
