; =============================================================================
; MemberToInheritanceRule
;
; MemberLink
;   B 
;   C
; |-
; InheritanceLink
;   B 
;   C
;
; -----------------------------------------------------------------------------

(define pln-rule-member-to-inheritance
	(BindLink
		(VariableList
			(VariableNode "$B")
			(VariableNode "$C"))
		(MemberLink
			(VariableNode "$B")
			(VariableNode "$C"))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm:pln-formula-member-to-inheritance")
			(ListLink
				(InheritanceLink
					(VariableNode "$B")
					(VariableNode "$C"))
				(MemberLink
					(VariableNode "$B")
					(VariableNode "$C"))))))

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
		(* (cog-stv-confidence MBC) 0.9)))

; =============================================================================
