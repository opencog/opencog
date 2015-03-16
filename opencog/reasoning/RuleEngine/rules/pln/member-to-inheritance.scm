; =============================================================================
; MemberToInheritanceRule
;	MemberLink( B (SatisfyingSetLink ( Variable $X 
;		(EvaluationLink (pred D (ListLink X C))))))
;			becomes
;	InheritanceLink( B (SatisfyingSetLink (Variable $X
; 		(EvaluationLink (pred D (ListLink X C))))))
; -----------------------------------------------------------------------------

(define pln-rule-member-to-inheritance
	(BindLink
		(ListLink
			(VariableNode "$B")
			(VariableNode "$C")
			(VariableNode "$D")
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
				(GroundedSchemaNode "scm:pln-formula-member-to-inheritance")
				(ListLink
					(InheritanceLink
						(VariableNode "$B")
						(SatisfyingSetLink
							(VariableNode "$X")
							(EvaluationLink
								(VariableNode "$D")
								(ListLink
									(VariableNode "$X")
									(VariableNode "$C")))))
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
; Member To Inheritance Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of the new link stays the same
; -----------------------------------------------------------------------------

(define (pln-formula-member-to-inheritance IBXDXC BXDXC)
	(cog-set-tv!
		IBXDXC
		(pln-formula-member-to-inheritance-side-effect-free
			IBXDXC
			BXDXC)))

; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (pln-formula-member-to-inheritance-side-effect-free IBXDXC BXDXC)
	(stv
		(cog-stv-strength BXDXC)
		(cog-stv-confidence BXDXC)))

; =============================================================================
