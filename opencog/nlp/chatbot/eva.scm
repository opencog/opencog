;
; eva.scm
;
; Hacky scaffolding for talking with Hanson Robotics Eva.
; Probably does not belong in this directory.

;--------------------------------------------------------------------
(define (var-decl var type)
	(TypedVariable (VariableNode var) (TypeNode type)))

;(define look-rule
;	(BindLink
;		(VariableList
;			(var-decl "$a-parse" "ParseNode")
;		)
;	)
;)

;--------------------------------------------------------------------
(define (imperative_process imp)
"
  Process imperative IMP, which should be a SentenceNode.

"
	(display imp)
	(newline)
)
