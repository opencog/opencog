; =============================================================================
; InheritanceToMemberRule
<<<<<<< HEAD
;	InheritanceLink( {B} C )
=======
;	InheritanceLink( B C )
>>>>>>> 69ea3d6728fe4179ead54fd57625473a5498704c
;			becomes
;	MemberLink( B C )
; -----------------------------------------------------------------------------

(define pln-rule-inheritance-to-member
	(BindLink
		(ListLink
			(VariableNode "$B")
			(VariableNode "$C"))
		(ImplicationLink
			(InheritanceLink
<<<<<<< HEAD
				(SetLink
					(VariableNode "$B"))
=======
				(VariableNode "$B")
>>>>>>> 69ea3d6728fe4179ead54fd57625473a5498704c
				(VariableNode "$C"))
			(ExecutionOutputLink
				(GroundedSchemaNode "scm:pln-formula-inheritance-to-member")
				(ListLink
					(MemberLink
						(VariableNode "$B")
						(VariableNode "$C"))
					(InheritanceLink
<<<<<<< HEAD
						(SetLink
							(VariableNode "$B"))
=======
						(VariableNode "$B")
>>>>>>> 69ea3d6728fe4179ead54fd57625473a5498704c
						(VariableNode "$C")))))))

; -----------------------------------------------------------------------------
; Inheritance To Member Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of the new link stays the same
; -----------------------------------------------------------------------------

(define (pln-formula-inheritance-to-member MBC IBC)
	(cog-set-tv!
		MBC
		(pln-formula-member-to-inheritance-side-effect-free
			MBC
			IBC)))

; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (pln-formula-inheritance-to-member-side-effect-free MBC IBC)
	(stv
		(cog-stv-strength IBC)
<<<<<<< HEAD
		(cog-stv-confidence IBC)))
=======
		(* (cog-stv-confidence IBC) 0.9))
>>>>>>> 69ea3d6728fe4179ead54fd57625473a5498704c

; =============================================================================
