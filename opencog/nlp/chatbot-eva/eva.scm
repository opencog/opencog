;
; eva.scm
;
; Hacky scaffolding for talking with Hanson Robotics Eva.

;--------------------------------------------------------------------
(use-modules (opencog) (opencog nlp) (opencog query))
(load "../relex2logic/rule-utils.scm")

; Global state for the current sentence.
(define current-sentence (AnchorNode "*-eva-current-sent-*"))
(StateLink current-sentence (SentenceNode "foobar"))

; Current imperative
(define current-imperative (AnchorNode "*-imperative-*"))
(StateLink current-imperative (WordNode "foobar"))

; Current action to be taken
(define current-action (AnchorNode "*-action-*"))
(StateLink current-action (WordNode "foobar"))

; ---------
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
		(State current-imperative (Variable "$direction"))
	)
)

; XXX temproary hack ...
(export look-rule-1)

;--------------------------------------------------------------------
; Global semantic knowledge

(define neutral-gaze
	(ListLink (Number 0) (Number 0) (Number 0)))

; Global state for the current look-at point
; This state records the direction that Eva is looking at,
; right now.
(StateLink (AnchorNode "head-pointing direction") neutral-gaze)
(StateLink (AnchorNode "gaze direction") neutral-gaze)

; Global knowledge about spatial directions
(DefineLink
	(DefinedSchema "rightwards")
	(ListLink ;; three numbers: x,y,z
		(Number 1)    ; x is forward
		(Number -0.5) ; y is right
		(Number 0)    ; z is up
	))

(DefineLink
	(DefinedSchema "leftwards")
	(ListLink ;; three numbers: x,y,z
		(Number 1)    ; x is forward
		(Number 0.5)  ; y is right
		(Number 0)    ; z is up
	))

(DefineLink
	(DefinedSchema "upwards")
	(ListLink ;; three numbers: x,y,z
		(Number 1)    ; x is forward
		(Number 0)    ; y is right
		(Number 0.3)  ; z is up
	))

(DefineLink
	(DefinedSchema "downwards")
	(ListLink ;; three numbers: x,y,z
		(Number 1)    ; x is forward
		(Number 0)    ; y is right
		(Number -0.3) ; z is up
	))

;--------------------------------------------------------------------
; Global knowledge about word-meaning

(ReferenceLink (WordNode "up") (DefinedSchema "upwards"))
(ReferenceLink (WordNode "down") (DefinedSchema "downwards"))
(ReferenceLink (WordNode "right") (DefinedSchema "rightwards"))
(ReferenceLink (WordNode "left") (DefinedSchema "leftwards"))

;--------------------------------------------------------------------
; Semantic disambiguation
; See if we know the meanings of things

(define look-semantics-rule-1
	(BindLink
		(VariableList
			(var-decl "$direction" "WordNode")
			(var-decl "$phys-ground" "DefinedSchemaNode")
		)
		(AndLink
			(StateLink current-imperative (Variable "$direction"))
			(ReferenceLink (Variable "$direction") (Variable "$phys-ground"))
		)
		(State current-action (Variable "$phys-ground"))
))

;--------------------------------------------------------------------
; Action schema

(define look-action-rule-1
	(BindLink
		(VariableList
			(var-decl "$action" "DefinedSchemaNode")
		)
		(AndLink
			(StateLink current-action (Variable "$action"))
		)
		(Evaluation (GroundedPredicate "py:look_at_point")
			(Variable "$action"))
))

;--------------------------------------------------------------------

(define (imperative-process imp)
"
  Process imperative IMP, which should be a SentenceNode.

"
	(StateLink current-sentence imp)
	(display (cog-bind look-rule-1))
	(display (cog-bind look-semantics-rule-1))
	(display (cog-bind look-action-rule-1))
	(newline)
)
