;==============================================================================
; Abduction Rule
;
; AND(Inheritance A C, Inheritance B C) entails Inheritance A B
;------------------------------------------------------------------------------

(define pln-rule-abduction
	(BindLink
		(ListLink
			(VariableNode "$A")
			(VariableNode "$B")
			(VariableNode "$C"))
		(ImplicationLink
			(AndLink
				(InheritanceLink
					(VariableNode "$A")
					(VariableNode "$C"))
				(InheritanceLink
					(VariableNode "$B")
					(VariableNode "$C"))
				(VariableNode "$A")
				(VariableNode "$B")
				(VariableNode "$C"))
			(ExecutionOutputLink
				(GroundedSchemaNode "scm: pln-formula-abduction")
				(ListLink
					(InheritanceLink
						(VariableNode "$A")
						(VariableNode "$B"))
					(InheritanceLink
						(VariableNode "$A")
						(VariableNode "$C"))
					(InheritanceLink
						(VariableNode "$B")
						(VariableNode "$C"))
					(VariableNode "$A")
					(VariableNode "$B")
					(VariableNode "$C"))))))
; -----------------------------------------------------------------------------
; Abduction Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of AB may be updated
; -----------------------------------------------------------------------------

(define (pln-formula-abduction AB AC BC A B C)
	(cog-set-tv!
		AB
		(pln-formula-abduction-side-effect-free
			AB
			AC
			BC
			A
			B
			C)))
; -----------------------------------------------------------------------------
; This version has no side effects and simply returns a TruthValue
; -----------------------------------------------------------------------------

(define (pln-formula-abduction-side-effect-free AB AC BC A B C)
	(let
		(
			(sCB 
				(/ 
					(* 
						(cog-stv-strength BC) 
						(cog-stv-strength C)
					) 
					(cog-stv-strength B)
				)
			)
			(sAC (cog-stv-strength AC))
			(sC (cog-stv-strength C))
		)
			(stv
				(if (eq? 0 (- 1 sAC))
					0 ; Set Strength to 0 if sNotB equals 0 to avoid
					  ; division by zero
					(+                         ; Strength
						(* sAC sCB)
						(/
							(*
								(-
									1
                                    sAC)
                                (-
                                    sCB
                                    (*
                                        sAC
                                        sCB)))
                            (-
                                1
                                sAC))))
                (min
                    (cog-stv-confidence AC)
                    (cog-stv-confidence BC))))) ; Confidence

; =============================================================================
