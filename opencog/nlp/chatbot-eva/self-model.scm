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

(use-modules (opencog nlp sureal))


; Rule-utils needed for defintion of var-decl, etc.
(load "../relex2logic/rule-utils.scm")

(define current-sentence (AnchorNode "*-eva-current-sent-*"))

;--------------------------------------------------------------------

(define neutral-gaze (Concept "forward"))

; Global state for head and eye-position self-awareness.
(StateLink (AnchorNode "*-head-direction-*") neutral-gaze)
(StateLink (AnchorNode "*-gaze-direction-*") neutral-gaze)

; Word-associations with state are already hard-coded in imperative.scm
;--------------------------------------------------------------------

; Recognize copular sentence "where are you looking?"
; This is insane overkill for the mere task of recognizing a sentence.
; The whole point of such complex analysis is to ...???
(define where-look-rule
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			(var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			(var-decl "$qvar-inst" "WordInstanceNode")
			(var-decl "$subj-inst" "WordInstanceNode")
			(var-decl "$direction" "ConceptNode")
		)
		(AndLink
			(StateLink current-sentence (Variable "$sent"))
			(parse-of-sent   "$parse" "$sent")
			(interp-of-parse "$interp" "$parse")
			(word-in-parse   "$verb-inst" "$parse")
			(word-in-parse   "$qvar-inst" "$parse")
			(LemmaLink (VariableNode "$qvar-inst") (WordNode "where"))
			(LemmaLink (VariableNode "$verb-inst") (WordNode "look"))
			(word-pos "$verb-inst" "verb")
			(dependency "_subj" "$verb-inst" "$subj-inst")
			(LemmaLink (VariableNode "$usbj-inst") (WordNode "you"))

			(State (Anchor "*-gaze-direction-*") (Variable "$direction"))
		)
		(SetLink
			(Evaluation (Predicate "looking") (ListLink (Concept "I")))
			(InheritanceLink
				(SatisfyingSet (Predicate "looking")) (Variable "$direction"))
		)
	)
)

;--------------------------------------------------------------------

; XXX We want to prime the atomspace with several response sentences,
; and the way to do that is to nlp-parse them here. The only problem is
; that this trips some crazy bug.  See bug #508
; https://github.com/opencog/atomspace/issues/508
; and so we have to do the load in run-chatbot.scm, instead. Arghhhh.
; XXX FIXME .. fix the above.
;
;(use-modules (opencog nlp relex2logic))
; (use-modules (opencog nlp chatbot))
; (load "../chatbot/chat-utils.scm")
;(load-r2l-rulebase)
;(nlp-parse "I am looking to the left")
;(nlp-parse "I am looking to the right")
;(nlp-parse "I am looking up")

;--------------------------------------------------------------------
(define (self-wh-query QUERY)
"
  Process a query about self.  Return an answer, or else nil, if
  no answer is known.  QUERY should be a SentenceNode.
"

	; Make the current sentence visible to everyone.
	(StateLink current-sentence QUERY)

	(let* ((r2l-set (cog-bind where-look-rule))
			(string-seq (sureal (car (cog-outgoing-set r2l-set))))
		)
(display r2l-set)
(display string-seq)
(newline)

		string-seq
	)
)
