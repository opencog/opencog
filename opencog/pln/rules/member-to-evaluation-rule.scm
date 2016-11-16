; =============================================================================
; TODO generalize MemberToEvaluationRule that works with any length argument
;
; MemberToEvaluationRule
;
; MemberLink
;   B
;   SatisfyingSetLink
;       X
;		EvaluationLink
;           D
;           ListLink
;               X
;               C
; |-
; EvaluationLink
;   D
;   ListLink
;       B
;       C
;


(load "formulas.scm")

; No ListLink, 1 argument in EvaluationLink
(define member-to-evaluation-0-rule
	(BindLink
		(VariableList
			(VariableNode "$B")
   			(TypedVariableLink
    				(VariableNode "$D")
    				(TypeNode "PredicateNode")))
		(MemberLink
			(VariableNode "$B")
			(SatisfyingSetLink
				(VariableNode "$X-M2E")
				(EvaluationLink
					(VariableNode "$D")
					(VariableNode "$X-M2E"))))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: member-to-evaluation-formula")
			(ListLink
				(EvaluationLink
					(VariableNode "$D")
					(VariableNode "$B"))
				(MemberLink
					(VariableNode "$B")
					(SatisfyingSetLink
						(VariableNode "$X-M2E")
						(EvaluationLink
							(VariableNode "$D")
							(VariableNode "$X-M2E"))))))))

; Has ListLink, 1 argument in EvaluationLink
(define member-to-evaluation-1-rule
	(BindLink
		(VariableList
			(VariableNode "$B")
   			(TypedVariableLink
    				(VariableNode "$D")
    				(TypeNode "PredicateNode")))
		(MemberLink
			(VariableNode "$B")
			(SatisfyingSetLink
				(VariableNode "$X-M2E")
				(EvaluationLink
					(VariableNode "$D")
					(ListLink
						(VariableNode "$X-M2E")))))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: member-to-evaluation-formula")
			(ListLink
				(EvaluationLink
					(VariableNode "$D")
					(ListLink
						(VariableNode "$B")))
				(MemberLink
					(VariableNode "$B")
					(SatisfyingSetLink
						(VariableNode "$X-M2E")
						(EvaluationLink
							(VariableNode "$D")
							(ListLink
								(VariableNode "$X-M2E")))))))))

; Has ListLink, 2 arguments in EvaluationLink, 1st argument in MemberLink
(define member-to-evaluation-2-1-rule
	(BindLink
		(VariableList
			(VariableNode "$B")
			(VariableNode "$C")
   			(TypedVariableLink
    				(VariableNode "$D")
    				(TypeNode "PredicateNode")))
		(MemberLink
			(VariableNode "$B")
			(SatisfyingSetLink
				(VariableNode "$X-M2E")
				(EvaluationLink
					(VariableNode "$D")
					(ListLink
						(VariableNode "$X-M2E")
						(VariableNode "$C")))))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: member-to-evaluation-formula")
			(ListLink
				(EvaluationLink
					(VariableNode "$D")
					(ListLink
						(VariableNode "$B")
						(VariableNode "$C")))
				(MemberLink
					(VariableNode "$B")
					(SatisfyingSetLink
						(VariableNode "$X-M2E")
						(EvaluationLink
							(VariableNode "$D")
							(ListLink
								(VariableNode "$X-M2E")
								(VariableNode "$C")))))))))

; Has ListLink, 2 arguments in EvaluationLink, 2nd argument in MemberLink
(define member-to-evaluation-2-2-rule
	(BindLink
		(VariableList
			(VariableNode "$B")
			(VariableNode "$C")
   			(TypedVariableLink
    				(VariableNode "$D")
    				(TypeNode "PredicateNode")))
		(MemberLink
			(VariableNode "$C")
			(SatisfyingSetLink
				(VariableNode "$X-M2E")
				(EvaluationLink
					(VariableNode "$D")
					(ListLink
						(VariableNode "$B")
						(VariableNode "$X-M2E")))))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: member-to-evaluation-formula")
			(ListLink
				(EvaluationLink
					(VariableNode "$D")
					(ListLink
						(VariableNode "$B")
						(VariableNode "$C")))
				(MemberLink
					(VariableNode "$C")
					(SatisfyingSetLink
						(VariableNode "$X-M2E")
						(EvaluationLink
							(VariableNode "$D")
							(ListLink
								(VariableNode "$B")
								(VariableNode "$X-M2E")))))))))



; -----------------------------------------------------------------------------
; Member To Evaluation Formula
; -----------------------------------------------------------------------------

(define (member-to-evaluation-formula EVAL MEM)
	(cog-set-tv! EVAL (cog-tv MEM)))


; Name the rule
(define member-to-evaluation-0-rule-name (DefinedSchemaNode "member-to-evaluation-0-rule"))
(DefineLink member-to-evaluation-0-rule-name member-to-evaluation-0-rule)

(define member-to-evaluation-1-rule-name (DefinedSchemaNode "member-to-evaluation-1-rule"))
(DefineLink member-to-evaluation-1-rule-name member-to-evaluation-1-rule)

(define member-to-evaluation-2-1-rule-name (DefinedSchemaNode "member-to-evaluation-2-1-rule"))
(DefineLink member-to-evaluation-2-1-rule-name member-to-evaluation-2-1-rule)

(define member-to-evaluation-2-2-rule-name (DefinedSchemaNode "member-to-evaluation-2-2-rule"))
(DefineLink member-to-evaluation-2-2-rule-name member-to-evaluation-2-2-rule)
