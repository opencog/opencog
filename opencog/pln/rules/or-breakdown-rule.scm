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


(define or-breakdown-rule
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
    (GroundedSchemaNode "scm: or-breakdown-formula")
    (ListLink
     (OrLink
      (VariableNode "$A")
      (VariableNode "$B"))
     (VariableNode "$A")
     (VariableNode "$B")))))

(define (or-breakdown-formula AB A B)
  (cog-set-tv!
   B
   (or-breakdown-side-effect-free-formula AB A B))
)

(define (or-breakdown-side-effect-free-formula AB A B)
  (let 
      ((sAB (cog-stv-strength AB))
       (cAB (cog-stv-confidence AB))
       (sA (cog-stv-strength A))
       (cA (cog-stv-confidence A)))
    (stv (/ sAB (- 1 sA)) (min cAB cA))))

; Name the rule
(define or-breakdown-rule-name (DefinedSchemaNode "or-breakdown-rule"))
(DefineLink or-breakdown-rule-name or-breakdown-rule)
