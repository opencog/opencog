;
; deduction.scm
;
; Implement simple crisp-logic deductive rules.
;
; Linas Vepstas August 2009
;
; -----------------------------------------------------------------------
; Make an implication link, which can, by pattern amtching, validate 
; the indicated hypothesis. The hypothesis is assumed to be a particular
; EvaluationLink
;
(define (make-simple-chain hypothesis)
	(let* ((oset (cog-outgoing-set hypothesis))
			(pred (car oset))            ; the PredicateNode
			(cand (cog-outgoing-set (cadr oset)))
			(evaluand-a (car cand))      ; predicand
			(evaluand-b (cadr cand))     ; predicand
			(var (VariableNode "$simple-chain-var"))
		)
		(VariableScopeLink
			var
			(ImplicationLink
				(AndLink
					(EvaluationLink
						pred
						(ListLink
							evaluand-a
							var
						)
					)
					(EvaluationLink
						pred
						(ListLink
							var
							evaluand-b
						)
					)
				)
				hypothesis
			)
		)
	)
)
