;
; deduction.scm
;
; Implement simple crisp-logic deductive rules.
;
; Linas Vepstas August 2009
;
; -----------------------------------------------------------------------
; Make an implication link, which can, by pattern matching, validate
; the indicated hypothesis. The hypothesis is assumed to be a particular
; EvaluationLink.  This essentially creates a simple deductive
; inheritance chain, i.e. if we want to prove P->Q then we explore
; all variabvles V such that P->V->Q. In opencog, the arrows -> are
; labelled by PredicateNodes. For example, consider Becky the cat.
; Then P==Becky Q==animal and the arrow -> is is-a, the hypothesis
; is then is-a(Becky,animal) or ..
;
;    EvaluationLink
;       PredicateNode "is-a"
;       ListLink
;          ConceptNode "Becky"
;          ConceptNode "animal"
;
; and this routine will create an ImplicationLink 
; involving is-a(Becky, $var) AND is-a($var, animal)
; which can then be fulfilled by $var==cat.
;
(define (make-simple-chain hypothesis)
	(let* ((oset (cog-outgoing-set hypothesis))
			(pred (car oset))            ; the PredicateNode
			(cand (cog-outgoing-set (cadr oset)))
			(evaluand-a (car cand))      ; predicand
			(evaluand-b (cadr cand))     ; predicand
		)
		(make-one-chain hypothesis evaluand-a evaluand-b pred pred)
	)
)

; -----------------------------------------------------------------------
; make-one-chain -- build a simple, single-step implication chain
; aka a "deduction rule". This is a simple, single-step deduction.
; The structure should be obvious by reading the code below!
;
(define (make-one-chain hypothesis evaluand-a evaluand-b pred-first pred-next)
	(define var (VariableNode "$simple-chain-var"))
	(BindLink
		var
		(AndLink
			(EvaluationLink
				pred-first
				(ListLink
					evaluand-a
					var
				)
			)
			(EvaluationLink
				pred-next
				(ListLink
					var
					evaluand-b
				)
			)
		)
		hypothesis
	)
)
