; =====================================================================
; NotEliminationRule
;
; NotLink 
;    A
; |-
;    A
;----------------------------------------------------------------------


(define not-elimination-rule
  (BindLink
   (VariableList
    (VariableNode "$A"))
   (NotLink
     (VariableNode "$A"))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: not-elimination-formula")
    (ListLink
     (NotLink
      (VariableNode "$A")
     (VariableNode "$A"))))))

(define (not-elimination-formula NA A)
  (cog-set-tv!
   A
   (not-elimination-side-effect-free-formula NA A))
)

(define (not-elimination-side-effect-free-formula NA A)
  (let 
      ((sNA (cog-stv-strength NA))
       (cNA (cog-stv-confidence NA)))
    (stv (- 1 sNA) cAB )))

; Name the rule
(define not-elimination-rule-name (DefinedSchemaNode "not-elimination-rule"))
(DefineLink not-elimination-rule-name not-elimination-rule)
