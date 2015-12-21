;
; eva.scm
;
; Hacky scaffolding for talking with Hanson Robotics Eva.

;--------------------------------------------------------------------
(use-modules (opencog) (opencog nlp))
(load "../relex2logic/rule-utils.scm")

(define (print-msg node) (display (cog-name node)) (newline) (stv 1 1))
(define (show-arg node) (display node) node)

; Handle short commands, such as "look up", "look left".
; Relex behaves very inconsistently, sometimes returning
; _advmod(look,left) and sometimes _to-be(look, right)
; when in both cases the correct result would be _to-do(look,left)
; So instead of trusting relex, we are just going to drop back
; to link-grammar, and look for the Pa link instead.
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
			(verb-tense "$verb-inst" "imperative")
			; (dependency "_advmod" "$verb-inst" "$direct-inst")
			; (dependency "_to-be" "$verb-inst" "$direct-inst")
			(lg-link "Pa" "$verb-inst" "$direct-inst")
			(word-lemma "$direct-inst" "$direction")
		)
		(ExecutionOutput
			; (GroundedSchema "py: hola")
			(GroundedSchema "scm: show-arg")
			(ListLink (Variable "$direction"))
		)
	)
)

(export look-rule-1)

;--------------------------------------------------------------------
(define (imperative_process imp)
"
  Process imperative IMP, which should be a SentenceNode.

"
	(display imp)
	(newline)
)
