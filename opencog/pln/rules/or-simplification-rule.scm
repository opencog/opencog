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


(define or-simplification-rule
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
    (GroundedSchemaNode "scm: or-simplification-formula")
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

(define (or-simplification-formula AABC ABC)
  (cog-set-tv!
   ABC
   (or-simplification-side-effect-free-formula AABC ABC))
)

(define (or-simplification-side-effect-free-formula AABC ABC)
  (let 
      ((sAABC (cog-stv-strength AABC))
       (cAABC (cog-stv-confidence AABC)))
    (stv sAABC cAABC)))

; Name the rule
(define or-simplification-rule-name (DefinedSchemaNode "or-simplification-rule"))
(DefineLink or-simplification-rule-name or-simplification-rule)
