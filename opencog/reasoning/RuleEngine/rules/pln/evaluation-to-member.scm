; ============================================================================= 
; GeneralEvaluationToMemberRule
;	EvaluationLink (pred D (ListLink B C))
;		becomes
;	MemberLink( B (SatisfyingSetLink( Variable $X 
;		(EvaluationLink (pred D (ListLink X C))))))
; -----------------------------------------------------------------------------

(define pln-rule-evaluation-to-member
	(BindLink
		(ListLink
			(VariableNode "$B")
			(VariableNode "$C")
			(TypedVariableLink
				(VariableNode "$D")
				(VariableTypeNode "PredicateNode")))
		(ImplicationLink
			(EvaluationLink
				(VariableNode "$D")
				(ListLink
					(VariableNode "$B")
					(VariableNode "$C")))
			(ExecutionOutputLink
				(GroundedSchemaNode "scm:pln-formula-evaluation-to-member")
				(ListLink
					(MemberLink
						(VariableNode "$B")
						(SatisfyingSetLink
							(VariableNode "$X")
							(EvaluationLink
								(VariableNode "$D")
								(ListLink
									(VariableNode "$X")
									(VariableNode "$C")))))
					(EvaluationLink
						(VariableNode "$D")
						(ListLink
							(VariableNode "$B")
							(VariableNode "$C"))))))))
; -----------------------------------------------------------------------------
; Evaluation To Member Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of the new link stays the same
; -----------------------------------------------------------------------------

(define (pln-formula-evaluation-to-member BXDXC DBC)
	(cog-set-tv!
		BXDXC
		(pln-formula-evaluation-to-member-side-effect-free
			BXDXC
			DBC)))
; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (pln-formula-evaluation-to-member-side-effect-free BXDXC DBC)
	(stv
		(cog-stv-strength DBC)
		(cog-stv-confidence DBC)))

; =============================================================================
