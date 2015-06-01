; =============================================================================
; MemberToInheritanceRule
;	MemberLink( B C )
;			becomes
<<<<<<< HEAD
;	InheritanceLink( {B} C )
=======
;	InheritanceLink( B C )
>>>>>>> 69ea3d6728fe4179ead54fd57625473a5498704c
; -----------------------------------------------------------------------------

(define pln-rule-member-to-inheritance
	(BindLink
		(ListLink
			(VariableNode "$B")
			(VariableNode "$C"))
		(ImplicationLink
			(MemberLink
				(VariableNode "$B")
				(VariableNode "$C"))
			(ExecutionOutputLink
				(GroundedSchemaNode "scm:pln-formula-member-to-inheritance")
				(ListLink
					(InheritanceLink
<<<<<<< HEAD
						(SetLink
							(VariableNode "$B"))
=======
						(VariableNode "$B")
>>>>>>> 69ea3d6728fe4179ead54fd57625473a5498704c
						(VariableNode "$C"))
					(MemberLink
						(VariableNode "$B")
						(VariableNode "$C")))))))

; -----------------------------------------------------------------------------
; Member To Inheritance Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of the new link stays the same
; -----------------------------------------------------------------------------

(define (pln-formula-member-to-inheritance IBC MBC)
	(cog-set-tv!
		IBC
		(pln-formula-member-to-inheritance-side-effect-free
			IBC
			MBC)))

; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (pln-formula-member-to-inheritance-side-effect-free IBC MBC)
	(stv
		(cog-stv-strength MBC)
<<<<<<< HEAD
		(cog-stv-confidence MBC)))
=======
		(* (cog-stv-confidence MBC) 0.9)))
>>>>>>> 69ea3d6728fe4179ead54fd57625473a5498704c

; =============================================================================
