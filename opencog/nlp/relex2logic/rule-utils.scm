;
; rule-tuils.scm
;
; Some generic rule utilities, not limited to r2l.
;
;--------------------------------------------------------------------
;
; Short-hand for declaring a variable.
(define (var-decl var type)
   (TypedVariableLink (VariableNode var) (TypeNode type)))

(define (word-in-parse word parse)
"  The WordInstanceNode WORD is in ParseNode PARSE. "
	(WordInstanceLink (VariableNode word) (VariableNode parse)))

(define (interp-of-parse interp parse)
"  The InterpretationNode INTERP is in ParseNode PARSE. "
	(InterpretationLink (VariableNode interp) (VariableNode parse)))

(define (dependency rel head dep)
"  RelEx dependency relation REL(HEAD, DEP) "
	(EvaluationLink
		(DefinedLinguisticRelationshipNode rel)
		(ListLink (VariableNode head) (VariableNode dep))))

(define (lg-link rel left right)
"  Link Grammar link LEFT-REL-RIGHT "
	(EvaluationLink
		(LinkGrammarRelationshipNode rel)
		(ListLink (VariableNode left) (VariableNode right))))
