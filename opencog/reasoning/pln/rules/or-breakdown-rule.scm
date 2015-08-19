; =====================================================================
; OrBreakdownRule
;
; OrLink 
;    A
;    B
; &
;    A
; |-
;    B
;----------------------------------------------------------------------


(define pln-rule-or-breakdown
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B"))
   (AndLink
    (OrLink
     (VariableNode "$A")
     (VariableNode "$B"))
    (VariableNode "$A"))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: pln-formula-or-breakdown")
    (ListLink
     (OrLink
      (VariableNode "$A")
      (VariableNode "$B"))
     (VariableNode "$A")
     (VariableNode "$B")))))

(define (pln-formula-or-breakdown AB A B)
  (cog-set-tv!
   B
   (pln-formula-or-breakdown-side-effect-free AB A B))
)

(define (pln-formula-or-breakdown-side-effect-free AB A B)
  (let 
      ((sAB (cog-stv-strength AB))
       (cAB (cog-stv-confidence AB))
       (sA (cog-stv-strength A))
       (cA (cog-stv-confidence A)))
    (stv (/ sAB (- 1 sA)) (min cAB cA))))

; Name the rule
(define pln-rule-or-breakdown-name (Node "pln-rule-or-breakdown"))
(DefineLink pln-rule-or-breakdown-name pln-rule-or-breakdown)
