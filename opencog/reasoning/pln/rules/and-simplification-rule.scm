; =====================================================================
; AndSimplificationRule
;
; AndLink 
;    A
;    AndLink
;       B
;	C
; |-
; AndLink
;    A
;    B
;    C
;----------------------------------------------------------------------


(define pln-rule-and-simplification
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
    (GroundedSchemaNode "scm: pln-formula-and-simplification")
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

(define (pln-formula-and-simplification AABC ABC)
  (cog-set-tv!
   ABC
   (pln-formula-and-simplification-side-effect-free AABC ABC))
)

(define (pln-formula-and-simplification-side-effect-free AABC ABC)
  (let 
      ((sAABC (cog-stv-strength AABC))
       (cAABC (cog-stv-confidence AABC)))
    (stv sAABC cAABC)))
