;==============================================================================
; Abduction Rule
;
; AND(Inheritance A C, Inheritance B C) entails Inheritance A B
;------------------------------------------------------------------------------
(load "formulas.scm")
(define abduction-rule
	(BindLink
		(VariableList
			(VariableNode "$A")
			(VariableNode "$B")
			(VariableNode "$C"))
		(AndLink
			(InheritanceLink
				(VariableNode "$A")
				(VariableNode "$C"))
			(InheritanceLink
				(VariableNode "$B")
				(VariableNode "$C"))
			; To avoid matching (Inheritance A C) and (Inheritance A C)
			(NotLink
				(EqualLink
					(VariableNode "$A")
					(VariableNode "$B"))))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: abduction-formula")
			(ListLink
				(InheritanceLink
					(VariableNode "$A")
					(VariableNode "$B"))
				(InheritanceLink
					(VariableNode "$A")
					(VariableNode "$C"))
				(InheritanceLink
					(VariableNode "$B")
					(VariableNode "$C"))))))

; -----------------------------------------------------------------------------
; Abduction Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of AB may be updated
; -----------------------------------------------------------------------------

(define (abduction-formula AB AC BC)
	(cog-set-tv!
		AB
		(abduction-side-effect-free-formula
			AB
			AC
			BC
			(gar AB)
			(gar BC)
			(gdr AC))))
; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (abduction-side-effect-free-formula AB AC BC A B C)
	(let
		(
			(sCB (inversion-strength-formula 
				(cog-stv-strength BC)
				(cog-stv-strength B) 
				(cog-stv-strength C)))
			(sAC (cog-stv-strength AC))
			(sC (cog-stv-strength C))
			(sB (cog-stv-strength B))
			(sA (cog-stv-strength A)))
		(stv 
			(simple-deduction-strength-formula sA sC sB sAC sCB) ;Strength
                	(min
                    		(cog-stv-confidence AB)
                    		(cog-stv-confidence BC))))) ; Confidence

; =============================================================================

; Name that rule
(define abduction-rule-name (Node "abduction-rule"))
(DefineLink abduction-rule-name abduction-rule)
