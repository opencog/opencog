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


(define or-elimination-rule
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B"))
   (AndLink
    (OrLink
     (VariableNode "$A")
     (VariableNode "$B")))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: or-elimination-formula")
    (ListLink
     (OrLink
      (VariableNode "$A")
      (VariableNode "$B"))
     (VariableNode "$A")
     (VariableNode "$B")))))

(define (or-elimination-formula AB A B)
  (cog-set-tv!
   A
   (or-elimination-side-effect-free-formula AB))
  (cog-set-tv!
   B
   (or-elimination-side-effect-free-formula AB)) 
)

(define (or-elimination-side-effect-free-formula AB)
  (let 
      ((sAB (cog-stv-strength AB))
       (cAB (cog-stv-confidence AB)))
    (stv (/ sAB 2) 1)))

; Name the rule
(define or-elimination-rule-name (DefinedSchemaNode "or-elimination-rule"))
(DefineLink or-elimination-rule-name or-elimination-rule)
