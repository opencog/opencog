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

(define and-elimination-rule
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B"))
   (AndLink
    (AndLink
     (VariableNode "$A")
     (VariableNode "$B")))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: and-elimination-formula")
    (ListLink
     (AndLink
      (VariableNode "$A")
      (VariableNode "$B"))
     (VariableNode "$A")
     (VariableNode "$B")))))

(define (and-elimination-formula AB A B)
  (cog-set-tv!
   A
   (and-elimination-side-effect-free-formula AB))
  (cog-set-tv!
   B
   (and-elimination-side-effect-free-formula AB)) 
)

(define (and-elimination-side-effect-free-formula AB)
  (let 
      ((sAB (cog-stv-strength AB))
       (cAB (cog-stv-confidence AB)))
    (stv (expt sAB 0.5) (/ cAB 1.42))))

; Name the rule
(define and-elimination-rule-name (DefinedSchemaNode "and-elimination-rule"))
(DefineLink and-elimination-rule-name and-elimination-rule)
