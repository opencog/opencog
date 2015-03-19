; ============================================================================= 
; GeneralEvaluationToMemberRule 
;	Takes EvaluationLinks with >=2 Arguments and creates a Member Link
;
;	EvaluationLink (pred D (ListLink B C))
;		becomes
;	MemberLink( B (SatisfyingSetLink( Variable $X 
;		(EvaluationLink (pred D (ListLink X C))))))
; -----------------------------------------------------------------------------

(define pln-rule-evaluation-to-member
	(BindLink
		(ListLink
			(VariableNode "$A")
			(TypedVariableLink
				(VariableNode "$D")
				(VariableTypeNode "PredicateNode")))
        (ImplicationLink
            (EvaluationLink
                (VariableNode "$D")
                (VariableNode "$A"))
            (ExecutionOutputLink
                (GroundedSchemaNode "scm:pln-formula-evaluation-to-member")
                (ListLink
                    (EvaluationLink
                        (VariableNode "$D")
                        (VariableNode "$A")))))))


; -----------------------------------------------------------------------------
; Evaluation To Member Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of the new link stays the same
; -----------------------------------------------------------------------------

(define (pln-formula-evaluation-to-member DBC)
	(MemberLink (stv (cog-stv-strength DBC) (cog-stv-confidence DBC))
		(car (cdr (cog-get-all-nodes DBC)))
		(SatisfyingSetLink
			(VariableNode "$X")
			(EvaluationLink
				(car (cog-get-all-nodes DBC))
				(ListLink
					(cons
						(VariableNode "$X")
						(cdr (cdr (cog-get-all-nodes DBC)))))))))

; =============================================================================
