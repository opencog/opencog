; ============================================================================= 
; GeneralMemberToEvaluationRule
;	MemberLink( B (SatisfyingSetLink( Variable $X 
;		(EvaluationLink (pred D (ListLink X C))))))
;		becomes
;	EvaluationLink (pred D (ListLink B C))
; -----------------------------------------------------------------------------

(define pln-rule-member-to-evaluation
	(BindLink
		(ListLink
			(VariableNode "$B")
			(VariableNode "$C")
   			(TypedVariableLink
    				(VariableNode "$D")
    				(VariableTypeNode "PredicateNode")))
		(ImplicationLink
			(MemberLink
				(VariableNode "$B")
				(SatisfyingSetLink
					(VariableNode "$X")
					(EvaluationLink
						(VariableNode "$D")
						(ListLink
							(VariableNode "$X")
							(VariableNode "$C")))))
			(ExecutionOutputLink
				(GroundedSchemaNode "scm:pln-formula-member-to-evaluation")
				(ListLink
					(EvaluationLink
						(VariableNode "$D")
						(ListLink
							(VariableNode "$B")
							(VariableNode "$C")))
					(MemberLink
						(VariableNode "$B")
						(SatisfyingSetLink
							(VariableNode "$X")
							(EvaluationLink
								(VariableNode "$D")
								(ListLink
									(VariableNode "$X")
									(VariableNode "$C"))))))))))

; -----------------------------------------------------------------------------
; Member To Evaluation Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of the new link stays the same
; -----------------------------------------------------------------------------

(define (pln-formula-member-to-evaluation DBC BXDXC)
	(cog-set-tv!
		DBC
		(pln-formula-member-to-evaluation-side-effect-free
			DBC
			BXDXC)))

; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (pln-formula-member-to-evaluation-side-effect-free DBC BXDXC)
	(stv
		(cog-stv-strength BXDXC)
		(cog-stv-confidence BXDXC)))

; =============================================================================
