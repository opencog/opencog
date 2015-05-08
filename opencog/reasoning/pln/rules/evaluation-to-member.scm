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

(include "formulas.scm")

(define pln-rule-evaluation-to-member
	(BindLink
		(VariableList
			(VariableNode "$A")
			(TypedVariableLink
				(VariableNode "$D")
				(TypeNode "PredicateNode")))
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
; Side-effect: TruthValue of the new link/s stays the same
; -----------------------------------------------------------------------------

(define (pln-formula-evaluation-to-member DA)
	(if (= (cog-arity (gdr DA)) 0)
		(MemberLink (stv (cog-stv-strength DA) (cog-stv-confidence DA))
			(gdr DA)
			(SatisfyingSetLink
				(VariableNode "$X")
				(EvaluationLink
					(gar DA)
					(VariableNode "$X"))))
		(ListLink
			(create-multiple-links DA '() (cog-outgoing-set (gdr DA))))))

(define (create-multiple-links DA preceding-nodes trailing-nodes)
	(if (not (null? trailing-nodes))
		(cons
			(MemberLink (stv (cog-stv-strength DA) (cog-stv-confidence DA))
				(car trailing-nodes)
				(SatisfyingSetLink
					(VariableNode "$X")
					(EvaluationLink
						(gar DA)
						(ListLink
							(append
								preceding-nodes
								(cons
									(VariableNode "$X")
									(cdr trailing-nodes)))))))
			(create-multiple-links 
				DA
				(reverse (cons (car trailing-nodes) (reverse preceding-nodes)))
				(cdr trailing-nodes)))
		'())) 

; =============================================================================
