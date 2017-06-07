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

(define inheritance-to-member-rule
	(BindLink
		(VariableList
			(VariableNode "$B")
			(VariableNode "$C"))
		(InheritanceLink
			(VariableNode "$B")
			(VariableNode "$C"))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: inheritance-to-member-formula")
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

(define (inheritance-to-member-formula MBC IBC)
	(cog-set-tv!
		MBC
		(member-to-inheritance-side-effect-free-formula
			MBC
			IBC)))

; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (inheritance-to-member-side-effect-free-formula MBC IBC)
	(stv
		(cog-stv-strength IBC)
		(* (cog-stv-confidence IBC) 0.9)))

; =============================================================================

; Name the rule
(define inheritance-to-member-rule-name
  (DefinedSchemaNode "inheritance-to-member-rule"))
(DefineLink
  inheritance-to-member-rule-name
  inheritance-to-member-rule)
