;
; eva.scm
;
; Hacky scaffolding for talking with Hanson Robotics Eva.

;--------------------------------------------------------------------
(use-modules (opencog) (opencog nlp) (opencog query))
(load "../relex2logic/rule-utils.scm")

; Global state for the current sentence.
(define current-sentence (AnchorNode "*-eva-current-sent-*"))

(define (print-msg node) (display (cog-name node)) (newline) (stv 1 1))

; XXX needs to be public, so that cog-bind can find this...
(define-public (show-arg node) (display node) node)

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
			(StateLink current-sentence (Variable "$sent"))
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
			(GroundedSchema "scm: show-arg")
			(ListLink (Variable "$direction"))
		)
	)
)

(export look-rule-1)

;--------------------------------------------------------------------
; Behaviors

(DefineLink
	(DefinedPredicate "look right")
	(Evaluation (GroundedPredicate "py:look_at_point")
		(ListLink ;; three numbers: x,y,z
			(Number 1)    ; x is forward
			(Number -0.5) ; y is right
			(Number 0)    ; z is up
		)))

(DefineLink
	(DefinedPredicate "look left")
	(Evaluation (GroundedPredicate "py:look_at_point")
		(ListLink ;; three numbers: x,y,z
			(Number 1)    ; x is forward
			(Number 0.5)  ; y is right
			(Number 0)    ; z is up
		)))

(DefineLink
	(DefinedPredicate "look up")
	(Evaluation (GroundedPredicate "py:look_at_point")
		(ListLink ;; three numbers: x,y,z
			(Number 1)    ; x is forward
			(Number 0)    ; y is right
			(Number 0.3)  ; z is up
		)))

(DefineLink
	(DefinedPredicate "look down")
	(Evaluation (GroundedPredicate "py:look_at_point")
		(ListLink ;; three numbers: x,y,z
			(Number 1)    ; x is forward
			(Number 0)    ; y is right
			(Number -0.3) ; z is up
		)))

;--------------------------------------------------------------------

(define (imperative-process imp)
"
  Process imperative IMP, which should be a SentenceNode.

"
	(StateLink current-sentence imp)
	(display (cog-bind look-rule-1))
	(newline)
)
