; =====================================================================
; AndBreakdownRule
;
; AndLink 
;    A
;    B
; &
;    A
; |-
;    B
;----------------------------------------------------------------------


(define pln-rule-and-breakdown
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B"))
   (AndLink
    (AndLink
     (VariableNode "$A")
     (VariableNode "$B"))
    (VariableNode "$A"))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: pln-formula-and-breakdown")
    (ListLink
     (AndLink
      (VariableNode "$A")
      (VariableNode "$B"))
     (VariableNode "$A")
     (VariableNode "$B")))))

(define (pln-formula-and-breakdown AB A B)
  (cog-set-tv!
   B
   (pln-formula-and-breakdown-side-effect-free AB A B))
)

(define (pln-formula-and-breakdown-side-effect-free AB A B)
  (let 
      ((sAB (cog-stv-strength AB))
       (cAB (cog-stv-confidence AB))
       (sA (cog-stv-strength A))
       (cA (cog-stv-confidence A)))
    (stv (/ sAB sA) (min cAB cA))))
