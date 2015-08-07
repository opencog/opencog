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
; pln-rule-evaluation-to-member-0
;	Converts EvaluationLinks which have only one member which is not part of 
;	the ListLink
;	eg:- EvaluationLink
;			PredicateNode "laughs"
;			ConceptNode "John"			
; -----------------------------------------------------------------------------

(define pln-rule-evaluation-to-member-0
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
			(GroundedSchemaNode "scm:pln-formula-evaluation-to-member-0")
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

(define (pln-formula-evaluation-to-member-0 MAXDX DA)
	(cog-set-tv! MAXDX
		(pln-formula-evaluation-to-member-side-effect-free
			MAXDX
			DA)))

; -----------------------------------------------------------------------------
; pln-rule-evaluation-to-member-1
;	Converts EvaluationLinks which have only one member which is part of 
;	the ListLink
;	eg:- EvaluationLink
;			PredicateNode "laughs"
;			ListLink
;				ConceptNode "John"			
; -----------------------------------------------------------------------------

(define pln-rule-evaluation-to-member-1
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
			(GroundedSchemaNode "scm:pln-formula-evaluation-to-member-1")
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

(define (pln-formula-evaluation-to-member-1 MAXDX DA)
	(cog-set-tv! MAXDX
		(pln-formula-evaluation-to-member-side-effect-free
			MAXDX
			DA)))

; -----------------------------------------------------------------------------
; pln-rule-evaluation-to-member-2
;	Converts EvaluationLinks which have two members
;	eg:- EvaluationLink
;			PredicateNode "Eat"
;			ListLink
;				ConceptNode "Jacob"
;				ConceptNode "Cookies"			
; -----------------------------------------------------------------------------

(define pln-rule-evaluation-to-member-2
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
			(GroundedSchemaNode "scm:pln-formula-evaluation-to-member-2")
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
							(VariableNode "$X")
							(EvaluationLink
								(VariableNode "$D")
								(ListLink
									(VariableNode "$A")
									(VariableNode "$X")))))
					(EvaluationLink
						(VariableNode "$D")
						(ListLink
							(VariableNode "$A")
							(VariableNode "$B")))))))


(define (pln-formula-evaluation-to-member-2 MAXDXB MBXDAX DAB)
	(cog-set-tv! MAXDXB
		(pln-formula-evaluation-to-member-side-effect-free
			MAXDXB
			DAB))
	(cog-set-tv! MBXDAX 
		(pln-formula-evaluation-to-member-side-effect-free
			MBXDAX
			DAB)))

(define (pln-formula-evaluation-to-member-side-effect-free MD ED)
	(stv
		(cog-stv-strength ED)
		(cog-stv-confidence ED)))

;(define pln-rule-evaluation-to-member
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
;           (GroundedSchemaNode "scm:pln-formula-evaluation-to-member")
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

;(define (pln-formula-evaluation-to-member DA)
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
(define pln-rule-evaluation-to-member-0-name
  (Node "pln-rule-evaluation-to-member-0"))
(DefineLink
  pln-rule-evaluation-to-member-0-name
  pln-rule-evaluation-to-member-0)

(define pln-rule-evaluation-to-member-1-name
  (Node "pln-rule-evaluation-to-member-1"))
(DefineLink
  pln-rule-evaluation-to-member-1-name
  pln-rule-evaluation-to-member-1)

(define pln-rule-evaluation-to-member-2-name
  (Node "pln-rule-evaluation-to-member-2"))
(DefineLink
  pln-rule-evaluation-to-member-2-name
  pln-rule-evaluation-to-member-2)
