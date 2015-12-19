;
; eva.scm
;
; Hacky scaffolding for talking with Hanson Robotics Eva.
; Probably does not belong in this directory.

;--------------------------------------------------------------------
(use-modules (opencog) (opencog nlp))
(load "../relex2logic/rule-utils.scm")

(define look-rule
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			(var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
		)
		(AndLink
			(parse-of-sent   "$parse" "$sent")
			(interp-of-parse "$interp" "$parse")
			(word-in-parse   "$verb-inst" "$parse")
			(LemmaLink (VariableNode "$verb-inst") (WordNode "look"))
			(word-pos "$verb-inst" "verb")
		)
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
