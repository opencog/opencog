; =====================================================================
; AndSimplificationRule
;
; A
; AndLink
;    B
;    C
; |-
; AndLink
;    A
;    B
;    C
;----------------------------------------------------------------------


(define and-simplification-rule
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B")
    (VariableNode "$C"))
   (AndLink
     (VariableNode "$A")
     (AndLink
      (VariableNode "$B")
      (VariableNode "$C")))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: and-simplification-formula")
    (ListLink
     (AndLink
      (VariableNode "$A")
      (AndLink
       (VariableNode "$B")
       (VariableNode "$C")))
     (AndLink
      (VariableNode "$A")
      (VariableNode "$B")
      (VariableNode "$C"))))))

(define (and-simplification-formula AABC ABC)
  (cog-set-tv!
   ABC
   (and-simplification-side-effect-free-formula AABC ABC))
)

(define (and-simplification-side-effect-free-formula AABC ABC)
  (let 
      ((sAABC (cog-stv-strength AABC))
       (cAABC (cog-stv-confidence AABC)))
    (stv sAABC cAABC)))

; Name the rule
(define and-simplification-rule-name
  (DefinedSchemaNode "and-simplification-rule"))
(DefineLink and-simplification-rule-name and-simplification-rule)
