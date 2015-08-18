; ============================================================================= 
; MemberToEvaluationRule
;
; MemberLink 
;   B 
;   SatisfyingSetLink
;       X 
;		EvaluationLink
;           D
;           ListLink 
;               X 
;               C
; |-
; EvaluationLink
;   D 
;   ListLink 
;       B 
;       C
;
; -----------------------------------------------------------------------------
; The position of X is not fixed in the evaluation link. Basically, the rule
; replaces X in the evaluation link with B.
; NOTE:- This rule can create issues with backward chaining since the new link
;        is created in the trailing function.

(include "formulas.scm")

(define pln-rule-member-to-evaluation
	(BindLink
		(VariableList
			(VariableNode "$B")
			(VariableNode "$C")
   			(TypedVariableLink
    				(VariableNode "$D")
    				(TypeNode "PredicateNode")))
		(MemberLink
			(VariableNode "$B")
			(SatisfyingSetLink
				(VariableNode "$X")
				(EvaluationLink
					(VariableNode "$D")
					(VariableNode "$C"))))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pln-formula-member-to-evaluation")
			(ListLink
				(MemberLink
					(VariableNode "$B")
					(SatisfyingSetLink
						(VariableNode "$X")
						(EvaluationLink
							(VariableNode "$D")
							(VariableNode "$C"))))))))

; -----------------------------------------------------------------------------
; Member To Evaluation Formula
;
; The formula checks if the argumentset of the evaluation link is n-ary or 
; unary. If it is unary, the first condition is satisfied and an evaluation
; link is created with the predicatenode and variable B. If the evaluation link
; is n-ary, find-replace function is called which replaces variable X in the 
; link element's list with variable B.
; -----------------------------------------------------------------------------



(define (pln-formula-member-to-evaluation BXDC)
	(if (= (cog-arity (gdddr BXDC)) 0)
		(EvaluationLink (stv (cog-stv-strength BXDC) (cog-stv-confidence BXDC))
			(gaddr BXDC)
			(gar BXDC))
		(EvaluationLink (stv (cog-stv-strength BXDC) (cog-stv-confidence BXDC))
			(gaddr BXDC)
			(ListLink
				(find-replace 
					(cog-outgoing-set (gdddr BXDC)) 
					(VariableNode "$X") 
					(gar BXDC))))))


; Name the rule
(define pln-rule-member-to-evaluation-name (Node "pln-rule-member-to-evaluation"))
(DefineLink pln-rule-member-to-evaluation-name pln-rule-member-to-evaluation)
