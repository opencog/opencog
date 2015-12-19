;
; eva.scm
;
; Hacky scaffolding for talking with Hanson Robotics Eva.
; Probably does not belong in this directory.

;--------------------------------------------------------------------
(define (var-decl var type)
	(TypedVariable (VariableNode var) (TypeNode type)))

(define look-rule
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
		)
		(parse-of-sent   "$parse" "$sent")
		(interp-of-parse "$interp" "$parse")
		(word-in-parse   "$verb-inst" "$parse")
		(LemmaLink (VariableNode "$verb-inst") (Word "look"))
		(word-pos "$verb-inst" "verb")
		(ExecutionOutput
			(GroundedSchema "py: hola")
			(ListLink)
		)
	)
)

;--------------------------------------------------------------------
(define (imperative_process imp)
"
  Process imperative IMP, which should be a SentenceNode.

"
	(display imp)
	(newline)
)
