; ============================================================================= 
; GeneralEvaluationToMemberRule 
;	Takes EvaluationLinks with >=2 Arguments and creates a Member Link
;
;	EvaluationLink (pred D (ListLink B C))
;		becomes
;	MemberLink( B (SatisfyingSetLink( Variable $X 
;		(EvaluationLink (pred D (ListLink X C))))))
;						&
;	MemberLink( B (LambdaLink( Variable $X 
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
; Side-effect: TruthValue of the new links stays the same
; -----------------------------------------------------------------------------

(define (pln-formula-evaluation-to-member DBC)
	(if (= (cog-arity (gdr DBC)) 0)
		(begin
			(MemberLink (stv (cog-stv-strength DBC) (cog-stv-confidence DBC))
				(gdr DBC)
				(SatisfyingSetLink
					(VariableNode "$X")
					(EvaluationLink
						(gar DBC)
						(VariableNode "$X"))))
			(MemberLink (stv (cog-stv-strength DBC) (cog-stv-confidence DBC))
    			(gdr DBC)
    			(LambdaLink
        			(VariableNode "$X")
        			(EvaluationLink
            			(gar DBC)
						(VariableNode "$X")))))
		(pln-create-member DBC '() (cog-outgoing-set (gdr DBC)))))

; -----------------------------------------------------------------------------
; Creating Multiple Links for Multiple Values
; -----------------------------------------------------------------------------

(define (pln-create-member DBC preceding-nodes trailing-nodes)
	(if (not (null? trailing-nodes))
		(begin
			(MemberLink (stv (cog-stv-strength DBC) (cog-stv-confidence DBC))
				(car trailing-nodes)
				(SatisfyingSetLink
					(VariableNode "$X")
					(EvaluationLink
						(gar DBC)
						(ListLink
							(append
								preceding-nodes
								(cons (VariableNode "$X")
									(cdr trailing-nodes)))))))
			(MemberLink (stv (cog-stv-strength DBC) (cog-stv-confidence DBC))
				(car trailing-nodes)
				(LambdaLink
					(VariableNode "$X")
					(EvaluationLink
						(gar DBC)
						(ListLink
							(append
								preceding-nodes
								(cons (VariableNode "$X")
									(cdr trailing-nodes)))))))
			(pln-create-member DBC 
				(reverse (cons (car trailing-nodes) (reverse preceding-nodes)))
				(cdr trailing-nodes)))))

; =============================================================================
