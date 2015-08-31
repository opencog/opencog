; =====================================================================
; NotEliminationRule
;
; NotLink 
;    A
; |-
;    A
;----------------------------------------------------------------------


(define pln-rule-not-elimination
  (BindLink
   (VariableList
    (VariableNode "$A"))
   (NotLink
     (VariableNode "$A"))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: pln-formula-not-elimination")
    (ListLink
     (NotLink
      (VariableNode "$A")
     (VariableNode "$A"))))))

(define (pln-formula-not-elimination NA A)
  (cog-set-tv!
   A
   (pln-formula-not-elimination-side-effect-free NA A))
)

(define (pln-formula-not-elimination-side-effect-free NA A)
  (let 
      ((sNA (cog-stv-strength NA))
       (cNA (cog-stv-confidence NA)))
    (stv (- 1 sNA) cAB )))

; Name the rule
(define pln-rule-not-elimination-name (Node "pln-rule-not-elimination"))
(DefineLink pln-rule-not-elimination-name pln-rule-not-elimination)
