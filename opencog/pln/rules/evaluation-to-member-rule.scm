; =============================================================================
; TODO:-GeneralEvaluationToMemberRule
; 
;	Takes EvaluationLinks and creates a Member Link
;
; EvaluationLink
;   D 
;   ListLink 
;       B 
;       C
; |-
; MemberLink
;   B 
;   SatisfyingSetLink
;       X 
;		EvaluationLink
;           D 
;           ListLink 
;               X 
;               C
;
; -----------------------------------------------------------------------------
; Temporary Rule for now. EvaluationToMemberRule split into three parts for 
; three different kinds of rules.
; -----------------------------------------------------------------------------

;(include "formulas.scm")

; -----------------------------------------------------------------------------
; evaluation-to-member-0-rule
;	Converts EvaluationLinks which have only one member which is not part of 
;	the ListLink
;	eg:- EvaluationLink
;			PredicateNode "laughs"
;			ConceptNode "John"			
; -----------------------------------------------------------------------------

(define evaluation-to-member-0-rule
	(BindLink
		(VariableList
			(TypedVariableLink
				(VariableNode "$A")
				(TypeNode "ConceptNode"))
			(TypedVariableLink
				(VariableNode "$D")
				(TypeNode "PredicateNode")))
		(EvaluationLink
			(VariableNode "$D")
			(VariableNode "$A"))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: evaluation-to-member-0-formula")
				(ListLink
					(MemberLink
						(VariableNode "$A")
						(SatisfyingSetLink
							(VariableNode "$X")
							(EvaluationLink
								(VariableNode "$D")
								(VariableNode "$X"))))
					(EvaluationLink
						(VariableNode "$D")
						(VariableNode "$A"))))))

(define (evaluation-to-member-0-formula MAXDX DA)
	(cog-set-tv! MAXDX
		(evaluation-to-member-side-effect-free-formula
			MAXDX
			DA)))

; -----------------------------------------------------------------------------
; evaluation-to-member-1-rule
;	Converts EvaluationLinks which have only one member which is part of 
;	the ListLink
;	eg:- EvaluationLink
;			PredicateNode "laughs"
;			ListLink
;				ConceptNode "John"			
; -----------------------------------------------------------------------------

(define evaluation-to-member-1-rule
	(BindLink
		(VariableList
			(VariableNode "$A")
			(TypedVariableLink
				(VariableNode "$D")
				(TypeNode "PredicateNode")))
		(EvaluationLink
			(VariableNode "$D")
			(ListLink
				(VariableNode "$A")))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: evaluation-to-member-1-formula")
				(ListLink
					(MemberLink
						(VariableNode "$A")
						(SatisfyingSetLink
							(VariableNode "$X")
							(EvaluationLink
								(VariableNode "$D")
								(ListLink
									(VariableNode "$X")))))
					(EvaluationLink
						(VariableNode "$D")
						(ListLink
							(VariableNode "$A")))))))

(define (evaluation-to-member-1-formula MAXDX DA)
	(cog-set-tv! MAXDX
		(evaluation-to-member-side-effect-free-formula
			MAXDX
			DA)))

; -----------------------------------------------------------------------------
; evaluation-to-member-2-rule
;	Converts EvaluationLinks which have two members
;	eg:- EvaluationLink
;			PredicateNode "Eat"
;			ListLink
;				ConceptNode "Jacob"
;				ConceptNode "Cookies"			
; -----------------------------------------------------------------------------

(define evaluation-to-member-2-rule
	(BindLink
		(VariableList
			(VariableNode "$A")
			(VariableNode "$B")
			(TypedVariableLink
				(VariableNode "$D")
				(TypeNode "PredicateNode")))
		(EvaluationLink
			(VariableNode "$D")
			(ListLink
				(VariableNode "$A")
				(VariableNode "$B")))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: evaluation-to-member-2-formula")
				(ListLink
					(MemberLink
						(VariableNode "$A")
						(SatisfyingSetLink
							(VariableNode "$X")
							(EvaluationLink
								(VariableNode "$D")
								(ListLink
									(VariableNode "$X")
									(VariableNode "$B")))))
					(MemberLink
						(VariableNode "$B")
						(SatisfyingSetLink
							(VariableNode "$Y")
							(EvaluationLink
								(VariableNode "$D")
								(ListLink
									(VariableNode "$A")
									(VariableNode "$Y")))))
					(EvaluationLink
						(VariableNode "$D")
						(ListLink
							(VariableNode "$A")
							(VariableNode "$B")))))))


(define (evaluation-to-member-2-formula MAXDXB MBXDAX DAB)
  (List
    (cog-set-tv! MAXDXB
		(evaluation-to-member-side-effect-free-formula
			MAXDXB
			DAB))
	(cog-set-tv! MBXDAX 
		(evaluation-to-member-side-effect-free-formula
			MBXDAX
			DAB))))

(define (evaluation-to-member-side-effect-free-formula MD ED)
	(stv
		(cog-stv-strength ED)
		(cog-stv-confidence ED)))

;(define evaluation-to-member-rule
;	(BindLink
;		(VariableList
;			(VariableNode "$A")
;			(TypedVariableLink
;				(VariableNode "$D")
;				(TypeNode "PredicateNode")))
;       (EvaluationLink
;           (VariableNode "$D")
;           (VariableNode "$A"))
;       (ExecutionOutputLink
;           (GroundedSchemaNode "scm: evaluation-to-member-formula")
;           (ListLink
;               (EvaluationLink
;                   (VariableNode "$D")
;                   (VariableNode "$A"))))))

; -----------------------------------------------------------------------------
; Evaluation To Member Formula
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Side-effect: TruthValue of the new link/s stays the same
; -----------------------------------------------------------------------------

;(define (evaluation-to-member-formula DA)
;	(if (= (cog-arity (gdr DA)) 0)
;		(MemberLink (stv (cog-stv-strength DA) (cog-stv-confidence DA))
;			(gdr DA)
;			(SatisfyingSetLink
;				(VariableNode "$X")
;				(EvaluationLink
;					(gar DA)
;					(VariableNode "$X"))))
;		(ListLink
;			(create-multiple-links DA '() (cog-outgoing-set (gdr DA))))))
;
;(define (create-multiple-links DA preceding-nodes trailing-nodes)
;	(if (not (null? trailing-nodes))
;		(cons
;			(MemberLink (stv (cog-stv-strength DA) (cog-stv-confidence DA))
;				(car trailing-nodes)
;				(SatisfyingSetLink
;					(VariableNode "$X")
;					(EvaluationLink
;						(gar DA)
;						(ListLink
;							(append
;								preceding-nodes
;								(cons
;									(VariableNode "$X")
;									(cdr trailing-nodes)))))))
;			(create-multiple-links 
;				DA
;				(reverse (cons (car trailing-nodes) (reverse preceding-nodes)))
;				(cdr trailing-nodes)))
;		'())) 

; =============================================================================

; Name the rule
(define evaluation-to-member-0-rule-name
  (DefinedSchemaNode "evaluation-to-member-0-rule"))
(DefineLink
  evaluation-to-member-0-rule-name
  evaluation-to-member-0-rule)

(define evaluation-to-member-1-rule-name
  (DefinedSchemaNode "evaluation-to-member-1-rule"))
(DefineLink
  evaluation-to-member-1-rule-name
  evaluation-to-member-1-rule)

(define evaluation-to-member-2-rule-name
  (DefinedSchemaNode "evaluation-to-member-2-rule"))
(DefineLink
  evaluation-to-member-2-rule-name
  evaluation-to-member-2-rule)
