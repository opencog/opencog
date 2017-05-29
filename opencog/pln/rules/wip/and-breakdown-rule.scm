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


(define and-breakdown-rule
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
    (GroundedSchemaNode "scm: and-breakdown-formula")
    (ListLink
     (AndLink
      (VariableNode "$A")
      (VariableNode "$B"))
     (VariableNode "$A")
     (VariableNode "$B")))))

(define (and-breakdown-formula AB A B)
  (cog-set-tv!
   B
   (and-breakdown-side-effect-free-formula AB A B))
)

(define (and-breakdown-side-effect-free-formula AB A B)
  (let 
      ((sAB (cog-stv-strength AB))
       (cAB (cog-stv-confidence AB))
       (sA (cog-stv-strength A))
       (cA (cog-stv-confidence A)))
    (stv (/ sAB sA) (min cAB cA))))

; Name the rule
(define and-breakdown-rule-name (DefinedSchemaNode "and-breakdown-rule"))
(DefineLink and-breakdown-rule-name and-breakdown-rule)
