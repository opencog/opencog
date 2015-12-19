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
	(WordInstanceLink (VariableNode word) (VariableNode parse)))

(define (dependency rel head dep)
	(EvaluationLink
		(DefinedLinguisticRelationshipNode rel)
		(ListLink (VariableNode head) (VariableNode dep))))
