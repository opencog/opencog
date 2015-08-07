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
;; Given rule is binary because creating more than one link would create
;; an issue with backward chaining.
;; TODO :- Create the rule n-ary

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

; Name the rule
(define pln-rule-and-elimination-name (Node "pln-rule-and-elimination"))
(DefineLink pln-rule-and-elimination-name pln-rule-and-elimination)
