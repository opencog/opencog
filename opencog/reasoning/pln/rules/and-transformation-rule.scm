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


(define pln-rule-and-transformation
  (BindLink
   (VariableList
    (VariableNode "$A")
    (VariableNode "$B"))
   (AndLink
     (VariableNode "$A")
     (VariableNode "$B"))
   (ExecutionOutputLink
    (GroundedSchemaNode "scm: pln-formula-and-transformation")
    (ListLink
     (AndLink
      (VariableNode "$A")
      (VariableNode "$B"))
     (NotLink 
      (ImplicationLink 
       (VariableNode "$A")
	(NotLink
	 (VariableNode "$B"))))))))

(define (pln-formula-and-transformation AB NIAB)
  (cog-set-tv!
   NIAB
   (pln-formula-and-transformation-side-effect-free AB NIAB))
)


(define (pln-formula-and-transformation-side-effect-free AB NIAB)
  (let 
      ((sAB (cog-stv-strength AB))
       (cAB (cog-stv-confidence AB)))
    (stv sAB cAB)))
