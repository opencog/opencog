; =====================================================================
; AndEliminationRule
;
; AndLink 
;    A
;    B
; |-
;    A
;    B
;----------------------------------------------------------------------


(define pln-rule-and-elimination
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B"))
   (AndLink
    (AndLink
     (VariableNode "$A")
     (VariableNode "$B")))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: pln-formula-and-elimination")
    (ListLink
     (AndLink
      (VariableNode "$A")
      (VariableNode "$B"))
     (VariableNode "$A")
     (VariableNode "$B")))))

(define (pln-formula-and-elimination AB A B)
  (cog-set-tv!
   A
   (pln-formula-and-elimination-side-effect-free AB))
  (cog-set-tv!
   B
   (pln-formula-and-elimination-side-effect-free AB)) 
)

(define (pln-formula-and-elimination-side-effect-free AB)
  (let 
      ((sAB (cog-stv-strength AB))
       (cAB (cog-stv-confidence AB)))
    (stv (expt sAB 0.5) (/ cAB 1.42))))
