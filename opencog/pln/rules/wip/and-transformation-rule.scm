; =====================================================================
; AndTransformationRule
;
; AndLink 
;    A
;    B
; |-
; (NotLink (A => (NotLink B)))
;
;----------------------------------------------------------------------


(define and-transformation-rule
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B"))
   (AndLink
     (VariableNode "$A")
     (VariableNode "$B"))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: and-transformation-formula")
    (ListLink
     (AndLink
      (VariableNode "$A")
      (VariableNode "$B"))
     (NotLink 
      (ImplicationLink 
       (VariableNode "$A")
	(NotLink
	 (VariableNode "$B"))))))))

(define (and-transformation-formula AB NIAB)
  (cog-set-tv!
   NIAB
   (and-transformation-side-effect-free-formula AB NIAB))
)


(define (and-transformation-side-effect-free-formula AB NIAB)
  (let 
      ((sAB (cog-stv-strength AB))
       (cAB (cog-stv-confidence AB)))
    (stv sAB cAB)))

; Name the rule
(define and-transformation-rule-name (DefinedSchemaNode "and-transformation-rule"))
(DefineLink and-transformation-rule-name and-transformation-rule)
