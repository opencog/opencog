; ============================================================================= 
; GeneralMemberToEvaluationRule
;	MemberLink( B (SatisfyingSetLink( Variable $X 
;		(EvaluationLink (pred D (ListLink X C))))))
;		becomes
;	EvaluationLink (pred D (ListLink B C))
; -----------------------------------------------------------------------------
(include "formulas.scm")

(define pln-rule-member-to-evaluation
	(BindLink
		(VariableList
			(VariableNode "$B")
			(VariableNode "$C")
   			(TypedVariableLink
    				(VariableNode "$D")
    				(TypeNode "PredicateNode")))
		(ImplicationLink
			(MemberLink
				(VariableNode "$B")
				(SatisfyingSetLink
					(VariableNode "$X")
					(EvaluationLink
						(VariableNode "$D")
						(VariableNode "$C"))))
			(ExecutionOutputLink
				(GroundedSchemaNode "scm:pln-formula-member-to-evaluation")
				(ListLink
					(MemberLink
						(VariableNode "$B")
						(SatisfyingSetLink
							(VariableNode "$X")
							(EvaluationLink
								(VariableNode "$D")
								(VariableNode "$C")))))))))

; -----------------------------------------------------------------------------
; Member To Evaluation Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of the new link stays the same
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

; =============================================================================
