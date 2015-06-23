; =============================================================================
; InheritanceToMemberRule
;
; InheritanceLink
;   B 
;   C
; |-
; MemberLink
;   B 
;   C
;
; -----------------------------------------------------------------------------

(define pln-rule-inheritance-to-member
	(BindLink
		(VariableList
			(VariableNode "$B")
			(VariableNode "$C"))
		(InheritanceLink
			(VariableNode "$B")
			(VariableNode "$C"))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm:pln-formula-inheritance-to-member")
			(ListLink
				(MemberLink
					(VariableNode "$B")
					(VariableNode "$C"))
				(InheritanceLink
					(VariableNode "$B")
					(VariableNode "$C"))))))

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
		(* (cog-stv-confidence IBC) 0.9)))

; =============================================================================
