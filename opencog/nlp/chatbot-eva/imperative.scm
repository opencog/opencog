;
; imperative.scm
;
; Scaffolding for converting English-language imperatives into
; robot actions.  This implements the full end-to-end pipeline,
; of converting English sentences to an intermediate form, matching
; the intermediate form to a grounded-knowledge base, and then
; performing the resulting action, if any.
;
; This handles imperative sentences, such as:
;
;     Eva, look to the right.
;     Look up!
;     Eva, turn to the right.
;     turn left
;     Look down.
;     look straight ahead
;     turn forward
;
;  ("looking" makes here move her eyes only; turning turns he whole head).
;
;     Smile!
;     frown
;     Look sad
;     Look happy
;     Look surprised
;     Look bored
;     Eva, express boredom
;     Eva, show happiness!
;
;
; This is a hard-code ad-hoc pipeline. Subject to change.
;

(use-modules (opencog) (opencog query) (opencog exec))

;--------------------------------------------------------------------
; State and state anchors. These should be thought of as work-arounds,
; to be replaced when a fine-tuned attention-allocation system is
; working.  The anchors serve to define the center of what the
; system is supposed to be "thinking about", right now -- the current
; sentence, the current action to be taken.

; Current action to be taken
(define current-action (AnchorNode "*-action-*"))
(StateLink current-action (WordNode "foobar"))

;--------------------------------------------------------------------
; Action schema
; This is wrong, but a hack for now.

; action-rule-ao uses the Action Orchestrator (in orchestrate.scm)
(define action-rule-ao
	(BindLink
		(VariableList
			(TypedVariable
				(Variable "$action")
				(Signature
					(EvaluationLink
						(Type "DefinedPredicateNode")
						(Type "ListLink"))))
		)
		(AndLink
			(ListLink current-action (Variable "$action"))
		)
		; (StateLink current-action (Variable "$action"))
		(Variable "$action")
))

;--------------------------------------------------------------------

; Stove-pipe hack to perform an action associated with an imperative.
(define-public (imperative-process imp)
"
  Process imperative IMP, which should be a SentenceNode.
"
	; Make the current sentence visible to everyone.
	(StateLink current-sentence imp)

	; Apply rules that analyze sentences -- if the current sentence
	; is an imperative of some sort, it will pick it apart into a
	; simplfied form, and glue the simplified from to an anchor.
	(cog-bind look-rule-1)
	(cog-bind look-rule-2)
	(cog-bind single-word-express-rule)
	(cog-bind single-word-gesture-rule)
	(cog-bind show-rule-1)
	(cog-bind show-rule-2)

	; Apply semantics-rule-1 -- if the current-imperative
	; anchor is a word we understand in a physical grounded
	; sense, then attach that sense to the current-action anchor.
	(cog-bind obj-semantics-rule-1-ao)

	(cog-bind obj-semantic-model-rule-1)
	(cog-bind obj-semantic-model-rule-2)

	; Perform the action, and print a reply.
	(let* ((act-do-do (cog-bind action-rule-ao))
			(action-list (cog-outgoing-set act-do-do))
		)
		; (display act-do-do) (newline)

		; Evaluate can throw if we give it bad args. Which happens during
		; development. So report any errors.
		(catch #t
			(lambda ()
				(for-each cog-evaluate! action-list))
			(lambda (key . args)
				(display "Exception: ") (display key) (newline)
				(display args) (newline)
				(display "Bad eval: ") (display act-do-do) (newline)))

		; At this time, a ListLink is used to anchor suggested
		; actions to the current-action anchor. Wipe these out.
		(for-each (lambda (x)
			(cog-extract-recursive (ListLink current-action x)))
				action-list)

		; XXX replace this by AIML or something.
		(if (eq? '() action-list)
			(begin
				(State (Anchor "Chatbot: ChatbotEvaAction")
					(Concept "Chatbot: NoResult"))
				(display "I don't know how to do that.\n")))

		(State (Anchor "Chatbot: ChatbotEva")
			(Concept "Chatbot: ProcessFinished"))
	)

	; Reset the current-imperative state, as otherwise, any subsequent
	; nonsense will get re-interpreted as the same action as before.
	(StateLink current-imperative (WordNode "foobar"))
	(StateLink current-action (WordNode "foobar"))

	; Set the return value to be #<unspecified>, which avoids printing
	; of the return value.  (if #f #f) has the same effect.
	*unspecified*
)

;--------------------------------------------------------------------
