; =====================================================================
; OrEliminationRule
;
; OrLink 
;    A
;    B
; |-
;    A
;    B
;----------------------------------------------------------------------
;; Given rule is binary because creating more than one link would create
;; an issue with backward chaining.
;; TODO :- Create the rule n-ary


(define pln-rule-or-elimination
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B"))
   (AndLink
    (OrLink
     (VariableNode "$A")
     (VariableNode "$B")))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: pln-formula-or-elimination")
    (ListLink
     (OrLink
      (VariableNode "$A")
      (VariableNode "$B"))
     (VariableNode "$A")
     (VariableNode "$B")))))

(define (pln-formula-or-elimination AB A B)
  (cog-set-tv!
   A
   (pln-formula-or-elimination-side-effect-free AB))
  (cog-set-tv!
   B
   (pln-formula-or-elimination-side-effect-free AB)) 
)

(define (pln-formula-or-elimination-side-effect-free AB)
  (let 
      ((sAB (cog-stv-strength AB))
       (cAB (cog-stv-confidence AB)))
    (stv (/ sAB 2) 1)))

; Name the rule
(define pln-rule-or-elimination-name (Node "pln-rule-or-elimination"))
(DefineLink pln-rule-or-elimination-name pln-rule-or-elimination)
