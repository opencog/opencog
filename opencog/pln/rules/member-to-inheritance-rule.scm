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

(define member-to-inheritance-rule
	(BindLink
		(VariableList
			(VariableNode "$B")
			(VariableNode "$C"))
		(MemberLink
			(VariableNode "$B")
			(VariableNode "$C"))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: member-to-inheritance-formula")
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

(define (member-to-inheritance-formula IBC MBC)
	(cog-set-tv!
		IBC
		(member-to-inheritance-side-effect-free-formula
			IBC
			MBC)))

; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (member-to-inheritance-side-effect-free-formula IBC MBC)
	(stv
		(cog-stv-strength MBC)
		(* (cog-stv-confidence MBC) 0.9)))

; =============================================================================

; Name the rule
(define member-to-inheritance-rule-name
  (DefinedSchemaNode "member-to-inheritance-rule"))
(DefineLink
  member-to-inheritance-rule-name
  member-to-inheritance-rule)
