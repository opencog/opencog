;
; isa.scm
;
; Experimental code for using PLN on IsA relationships
;

; Define the meaning of "isa" relations
; If IsA(A,B) and IsA (B,C) then IsA(A,C)
(ForAllLink (stv 1 1)
	(ListLink (stv 1 0)
		(VariableNode "$var-isa-A" (stv 1 0))
		(VariableNode "$var-isa-B" (stv 1 0))
		(VariableNode "$var-isa-C" (stv 1 0))
	)
	(ImplicationLink (stv 1 0)
		; "If ..." part of the implcation
		(AndLink (stv 1 0)
			(EvaluationLink (stv 1 0)
				(DefinedLinguisticRelationshipNode "isa")
				(ListLink (stv 1 0)
					(VariableNode "$var-isa-A")
					(VariableNode "$var-isa-B")
				)
			)
			(EvaluationLink (stv 1 0)
				(DefinedLinguisticRelationshipNode "isa")
				(ListLink (stv 1 0)
					(VariableNode "$var-isa-B")
					(VariableNode "$var-isa-C")
				)
			)
		)
		; "then ..." part of the implication
		(EvaluationLink (stv 1 0)
			(DefinedLinguisticRelationshipNode "isa")
			(ListLink (stv 1 0)
				(VariableNode "$var-isa-A")
				(VariableNode "$var-isa-C")
			)
		)
	)
)
