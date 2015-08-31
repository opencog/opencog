; =====================================================================
; NotSimplificationRule
;
; NotLink 
;    NotLink
;       A
; |-
;    A
;----------------------------------------------------------------------


(define pln-rule-not-simplification
  (BindLink
   (VariableList
    (VariableNode "$A"))
   (NotLink
    (NotLink
     (VariableNode "$A")))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: pln-formula-not-simplification")
    (ListLink
     (NotLink
      (NotLink
       (VariableNode "$A")))
     (VariableNode "$A")))))

(define (pln-formula-not-simplification NNA A)
  (cog-set-tv!
   A
   (pln-formula-not-simplification-side-effect-free NNA A))
)

(define (pln-formula-not-simplification-side-effect-free NNA A)
  (let 
      ((sNNA (cog-stv-strength NNA))
       (cNNA (cog-stv-confidence NNA)))
    (stv (- 1 sNNA) cNNA)))

; Name the rule
(define pln-rule-not-simplification-name
  (Node "pln-rule-not-simplification"))
(DefineLink
  pln-rule-not-simplification-name
  pln-rule-not-simplification)
