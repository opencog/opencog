; =====================================================================
; NotRule
; (http://wiki.opencog.org/w/NotRule TODO)
;
; A<TV1>
; |-
; NotLink <TV>
;    A
;----------------------------------------------------------------------

(define pln-rule-not
  (BindLink
    (VariableList
      (VariableNode "$A"))
    (ImplicationLink
      (VariableNode "$A")
      (ExecutionOutputLink
        (GroundedSchemaNode "scm: pln-formula-not")
        (ListLink
		  (NotLink
			  (VariableNode "$A"))
          (VariableNode "$A"))))))

(define (pln-formula-not nA A)
  (if (eq? (cog-type A) 'NotLink)
	(cog-set-tv!
		(gar A)
		(pln-formula-not-side-effect-free A))
	(cog-set-tv!
		nA
		(pln-formula-not-side-effect-free A))))

(define (negate-strength x)
  (- 1 x))

(define (pln-formula-not-side-effect-free A)
  (let ((sA (cog-stv-strength A))
        (cA (cog-stv-confidence A)))
    (stv (negate-strength sA) cA)))

