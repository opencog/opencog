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


(include "formulas.scm")

; No ListLink, 1 argument in EvaluationLink
(define pln-rule-member-to-evaluation-0
	(BindLink
		(VariableList
			(VariableNode "$B")
   			(TypedVariableLink
    				(VariableNode "$D")
    				(TypeNode "PredicateNode"))
    			(TypedVariableLink
    				(VariableNode "$X-M2E")
    				(TypeNode "VariableNode")))
		(MemberLink
			(VariableNode "$B")
			(SatisfyingSetLink
				(VariableNode "$X-M2E")
				(EvaluationLink
					(VariableNode "$D")
					(VariableNode "$X-M2E"))))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pln-formula-member-to-evaluation")
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
(define pln-rule-member-to-evaluation-1
	(BindLink
		(VariableList
			(VariableNode "$B")
   			(TypedVariableLink
    				(VariableNode "$D")
    				(TypeNode "PredicateNode"))
    			(TypedVariableLink
    				(VariableNode "$X-M2E")
    				(TypeNode "VariableNode")))
		(MemberLink
			(VariableNode "$B")
			(SatisfyingSetLink
				(VariableNode "$X-M2E")
				(EvaluationLink
					(VariableNode "$D")
					(ListLink
						(VariableNode "$X-M2E")))))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pln-formula-member-to-evaluation")
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
(define pln-rule-member-to-evaluation-2-1
	(BindLink
		(VariableList
			(VariableNode "$B")
			(VariableNode "$C")
   			(TypedVariableLink
    				(VariableNode "$D")
    				(TypeNode "PredicateNode"))
    			(TypedVariableLink
    				(VariableNode "$X-M2E")
    				(TypeNode "VariableNode")))
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
			(GroundedSchemaNode "scm: pln-formula-member-to-evaluation")
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
(define pln-rule-member-to-evaluation-2-2
	(BindLink
		(VariableList
			(VariableNode "$B")
			(VariableNode "$C")
   			(TypedVariableLink
    				(VariableNode "$D")
    				(TypeNode "PredicateNode"))
    			(TypedVariableLink
    				(VariableNode "$X-M2E")
    				(TypeNode "VariableNode")))
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
			(GroundedSchemaNode "scm: pln-formula-member-to-evaluation")
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

(define (pln-formula-member-to-evaluation EVAL MEM)
	(cog-set-tv! EVAL (cog-tv MEM)))


; Name the rule
(define pln-rule-member-to-evaluation-0-name (Node "pln-rule-member-to-evaluation-0"))
(DefineLink pln-rule-member-to-evaluation-0-name pln-rule-member-to-evaluation-0)

(define pln-rule-member-to-evaluation-1-name (Node "pln-rule-member-to-evaluation-1"))
(DefineLink pln-rule-member-to-evaluation-1-name pln-rule-member-to-evaluation-1)

(define pln-rule-member-to-evaluation-2-1-name (Node "pln-rule-member-to-evaluation-2-1"))
(DefineLink pln-rule-member-to-evaluation-2-1-name pln-rule-member-to-evaluation-2-1)

(define pln-rule-member-to-evaluation-2-2-name (Node "pln-rule-member-to-evaluation-2-2"))
(DefineLink pln-rule-member-to-evaluation-2-2-name pln-rule-member-to-evaluation-2-2)
