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

(define (word-inst word parse)
	(WordInstanceLink (VariableNode word) (VariableNode parse))
