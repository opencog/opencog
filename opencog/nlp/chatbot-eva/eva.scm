;
; eva.scm
;
; Hacky scaffolding for talking with Hanson Robotics Eva.

;--------------------------------------------------------------------
(use-modules (opencog) (opencog nlp))
(load "../relex2logic/rule-utils.scm")

(define (print-msg node) (display (cog-name node)) (newline) (stv 1 1))
(define (show-arg node) (display node) node)

; Handle _advmod(look, $direction)
(define look-rule-1
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			(var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			(var-decl "$direct-inst" "WordInstanceNode")
			(var-decl "$direction" "WordNode")
		)
		(AndLink
			(parse-of-sent   "$parse" "$sent")
			(interp-of-parse "$interp" "$parse")
			(word-in-parse   "$verb-inst" "$parse")
			(LemmaLink (VariableNode "$verb-inst") (WordNode "look"))
			(word-pos "$verb-inst" "verb")
			(dependency "_advmod" "$verb-inst" "$direct-inst")
			(word-lemma "$direct-inst" "$direction")
		)
		(ExecutionOutput
			; (GroundedSchema "py: hola")
			(GroundedSchema "scm: show-arg")
			(ListLink (Variable "$direction"))
		)
	)
)

(define look-rule-2
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			(var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			(var-decl "$direct-inst" "WordInstanceNode")
			(var-decl "$direction" "WordNode")
		)
		(AndLink
			(parse-of-sent   "$parse" "$sent")
			(interp-of-parse "$interp" "$parse")
			(word-in-parse   "$verb-inst" "$parse")
			(LemmaLink (VariableNode "$verb-inst") (WordNode "look"))
			(word-pos "$verb-inst" "verb")
			(dependency "_advmod" "$verb-inst" "$direct-inst")
			(word-lemma "$direct-inst" "$direction")
		)
		(ExecutionOutput
			; (GroundedSchema "py: hola")
			(GroundedSchema "scm: show-arg")
			(ListLink (Variable "$direction"))
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
