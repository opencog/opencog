; =============================================================================
; InheritanceToMemberRule
;	InheritanceLink( B (SatisfyingSetLink (Variable $X
; 		(EvaluationLink (pred D (ListLink X C))))))
;			becomes
;	MemberLink( B (SatisfyingSetLink ( Variable $X 
;		(EvaluationLink (pred D (ListLink X C))))))
; -----------------------------------------------------------------------------

(define pln-rule-inheritance-to-member
	(BindLink
		(ListLink
			(VariableNode "$B")
			(VariableNode "$C")
			(VariableNode "$D")
			(TypedVariableLink
				(VariableNode "$D")
				(VariableTypeNode "PredicateNode")))
		(ImplicationLink
			(InheritanceLink
				(VariableNode "$B")
				(SatisfyingSetLink
					(VariableNode "$X")
					(EvaluationLink
						(VariableNode "$D")
						(ListLink
							(VariableNode "$X")
							(VariableNode "$C")))))
			(ExecutionOutputLink
				(GroundedSchemaNode "scm:pln-formula-inheritance-to-member")
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
					(InheritanceLink
						(VariableNode "$B")
						(SatisfyingSetLink
							(VariableNode "$X")
							(EvaluationLink
								(VariableNode "$D")
								(ListLink
									(VariableNode "$X")
									(VariableNode "$C"))))))))))

; -----------------------------------------------------------------------------
; Inheritance To Member Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of the new link stays the same
; -----------------------------------------------------------------------------

(define (pln-formula-inheritance-to-member BXDXC IBXDXC)
	(cog-set-tv!
		BXDXC
		(pln-formula-member-to-inheritance-side-effect-free
			BXDXC
			IBXDXC)))

; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (pln-formula-inheritance-to-member-side-effect-free BXDXC IBXDXC)
	(stv
		(cog-stv-strength IBXDXC)
		(cog-stv-confidence IBXDXC)))

; =============================================================================
