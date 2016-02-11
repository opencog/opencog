;
; self-model.scm
;
; Model of the robot-self, in the atomspace.
;
; In order for the robot to be able to talk about itself, it must have
; some amount of self-awareness; specifically, of its current physical
; state (head and eye position) and some memory of its most recent
; actions. That model-of-self must lie in the atomspace, where it can
; be examined during the course of linguistic interaction.  It is a
; "model" -- it is NOT the actual blender rig; it is not the actual
; mechanical motor positions and velocities.
;
; This is meant to be exemplary: there also needs to be a model of
; Eva's environment (including the people that she sees in that
; environment), so that she can also talk about that.

(use-modules  (opencog nlp sureal))

; Rule-utils needed for defintion of var-decl, etc.
(load "../relex2logic/rule-utils.scm")

(define current-sentence (AnchorNode "*-eva-current-sent-*"))

;--------------------------------------------------------------------

(define neutral-gaze (DefinedSchema "forwards"))

; Global state for head and eye-position self-awareness.
(StateLink (AnchorNode "*-head-direction-*") neutral-gaze)
(StateLink (AnchorNode "*-gaze-direction-*") neutral-gaze)

; Word-associations with state are already hard-coded in imperative.scm
;--------------------------------------------------------------------

(define where-look-rule
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			(var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			; (var-decl "$direct-inst" "WordInstanceNode")
			; (var-decl "$direction" "WordNode")
		)
		(AndLink
			(StateLink current-sentence (Variable "$sent"))
			(parse-of-sent   "$parse" "$sent")
			(interp-of-parse "$interp" "$parse")
			(word-in-parse   "$verb-inst" "$parse")
			(LemmaLink (VariableNode "$verb-inst") (WordNode "look"))
			(word-pos "$verb-inst" "verb")
		)
		(ListLink
			(Variable "$verb-inst")
		)
	)
)

;--------------------------------------------------------------------
(define (self-wh-query QUERY)
"
  Process a query about self.  Return an answer, or else nil, if
  no answer is known.  QUERY should be a SentenceNode.
"

	; Make the current sentence visible to everyone.
	(StateLink current-sentence QUERY)

(display
	(cog-bind where-look-rule)
)

	(list (list "foobar"))
)
