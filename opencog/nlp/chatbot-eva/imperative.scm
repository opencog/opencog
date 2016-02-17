;
; imperative.scm
;
; Scaffolding for converting English-langauge imperatives into
; robot actions.

; Rule-utils needed for defintion of var-decl, etc.
(load "../relex2logic/rule-utils.scm")

;--------------------------------------------------------------------
; State and state anchors. These should be thought of as work-arounds,
; to be replaced when a fine-tuned attention-allocation system is
; working.  The anchors serve to define the center of what the
; system is supposed to be "thinking about", right now -- the current
; sentence, the current action to be taken.

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

; ---------------------------------------------------------------
; Handle short imperative commands, such as "look up", "look left".
; This is a rule, meant to be applied to the current sentence.
; It will extract a direction to look at, and it will set
; the current-imperative state to that direction.
;
; Relex behaves very inconsistently, sometimes returning
; _advmod(look,left) and sometimes _to-be(look, right)
; when in both cases the correct result would be _to-do(look,left)
; So instead of trusting relex, we are just going to drop back
; to link-grammar, and look for the MVa/Pa link instead.
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
			(ChoiceLink
				(lg-link "MVa" "$verb-inst" "$direct-inst")
				(lg-link "Pa" "$verb-inst" "$direct-inst"))
			(word-lemma "$direct-inst" "$direction")
		)
		(State current-imperative
			(ActionLink
				(WordNode "look") ;; SchemaNode, the verb.
				(ListLink (Variable "$direction"))
		))
	)
)

; Matches sentences of the form "look to the right" and
; "look to the left".
(define look-rule-2
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			(var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			(var-decl "$prep-inst" "WordInstanceNode")
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
			; Specific LG linkage of
			; look >-MVp-> to >-Ju-> direction
			(lg-link "MVp" "$verb-inst" "$prep-inst")
			(ChoiceLink
				(lg-link "Js" "$prep-inst" "$direct-inst")
				(lg-link "Ju" "$prep-inst" "$direct-inst"))

			(word-lemma "$direct-inst" "$direction")
		)
		(State current-imperative
			(ActionLink
				(WordNode "look") ;; SchemaNode, the verb.
				(ListLink (Variable "$direction"))
		))
	)
)

; Abstracted variant of the above BindLinks. Given an explicit
; verb, this rule picks out ways in which an object attaches to
; the verb. The idea of 'object' is used loosely, here: it can
; a prepostional object, direct object, predicative adjective, etc.
;
; Note that much of this is boilerplate: the SentenceNode, ParseNode,
; and InterpretationNode: this is all more or less cruft, used to make
; up for the fact that attention allocation does not yet work correctly.
; This boilerplate is merely trying to ensure that all the words we
; look at are from the same sentence.  Surely, there must be an easier
; way to ensure connectivity?
(define (imperative-object-rule-template VERB-WORD DECL LINKS)
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			(var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			DECL
			(var-decl "$obj-inst" "WordInstanceNode")
			(var-decl "$object" "WordNode")
		)
		(AndLink
			(StateLink current-sentence (Variable "$sent"))
			(parse-of-sent   "$parse" "$sent")
			(interp-of-parse "$interp" "$parse")
			(word-in-parse   "$verb-inst" "$parse")
			(LemmaLink (VariableNode "$verb-inst") VERB-WORD)
			(word-pos "$verb-inst" "verb")
			(verb-tense "$verb-inst" "imperative")
			; Specific LG linkage
			LINKS
			(word-lemma "$obj-inst" "$object")
		)
		(State current-imperative
			(ActionLink
				VERB-WORD
				(ListLink (Variable "$object"))
		))
	)
)

; Re-implementation of look-rule-1 and 2, using the shorter template.
(define look-rule-1
	(imperative-object-rule-template
		(WordNode "look")  ; VERB-WORD
		'()                ; DECL
		(ChoiceLink        ; LINKS
			(lg-link "MVa" "$verb-inst" "$obj-inst")
			(lg-link "MVp" "$verb-inst" "$obj-inst")
			(lg-link "Pa" "$verb-inst" "$obj-inst"))
	))

(define look-rule-2
	(imperative-object-rule-template
		(WordNode "look")                           ; VERB-WORD
		(var-decl "$prep-inst" "WordInstanceNode")  ; DECL
		(list ; turn --MVp-> to --Ju-> object    ; LINKS
			(lg-link "MVp" "$verb-inst" "$prep-inst")
			(ChoiceLink
				(lg-link "Js" "$prep-inst" "$obj-inst")
				(lg-link "Ju" "$prep-inst" "$obj-inst"))
		)
	))

; Same as look-rule-1 and look-rule-2 but with verb "turn"
(define turn-rule-3
	(imperative-object-rule-template
		(WordNode "turn")  ; VERB-WORD
		'()                ; DECL
		(ChoiceLink        ; LINKS
			(lg-link "MVa" "$verb-inst" "$obj-inst")
			(lg-link "Pa" "$verb-inst" "$obj-inst"))
	))

(define turn-rule-4
	(imperative-object-rule-template
		(WordNode "turn")                           ; VERB-WORD
		(var-decl "$prep-inst" "WordInstanceNode")  ; DECL
		(list ; turn --MVp-> to --Ju-> object    ; LINKS
			(lg-link "MVp" "$verb-inst" "$prep-inst")
			(ChoiceLink
				(lg-link "Js" "$prep-inst" "$obj-inst")
				(lg-link "Ju" "$prep-inst" "$obj-inst"))
		)
	))

; Design notes:
; Rather than hand-crafting a bunch of rules like the above, we should
; do two things:
; (1) implement fuzzy matching, so that anything vaguely close to the
;     desired imperative will get matched.
; (2) implement automated learning of new rules, and refinement of
;     existing rules.

;--------------------------------------------------------------------

; Handle verbal expression imperatives, e.g. Smile! Frown!
(define (imperative-express-verb-template VERB-WORD)
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			(var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
		)
		(AndLink
			(StateLink current-sentence (Variable "$sent"))
			(parse-of-sent   "$parse" "$sent")
			(interp-of-parse "$interp" "$parse")
			(word-in-parse   "$verb-inst" "$parse")
			(word-pos "$verb-inst" "verb")
			(verb-tense "$verb-inst" "imperative")
			(LemmaLink (VariableNode "$verb-inst") VERB-WORD)
		)
		(State current-imperative
			(ActionLink
				(WordNode "express") ;; SchemaNode, the verb.
				(ListLink VERB-WORD)
		))
	)
)

(define smile-rule
	(imperative-express-verb-template (WordNode "smile")))
(define frown-rule
	(imperative-express-verb-template (WordNode "frown")))
(define recoil-rule
	(imperative-express-verb-template (WordNode "recoil")))

(define show-rule-1
	(imperative-object-rule-template
		(WordNode "show")         ; VERB-WORD
		'()                       ; DECL
		(lg-link "Ou" "$verb-inst" "$obj-inst") ; LINKS
	))

(define show-rule-2
	(imperative-object-rule-template
		(WordNode "express")      ; VERB-WORD
		'()                       ; DECL
		(lg-link "Ou" "$verb-inst" "$obj-inst") ; LINKS
	))


;--------------------------------------------------------------------
; Semantic disambiguation.
;
; See if we know the meaning of utterances.
; These are rules to be applied to the current state: if the
; current word/utterance has an explicit concrete grounding, then
; construct the grounded equivalent.

; obj-semantics-rule-1 -- if the current "simplified-English" imperative
; contains a verb that we know, and an "object" it refers to, then suggest
; a grounded action.  Here, 'object' is used loosely: it can be a direct
; object, a prepositional object, or even an adjective. For example,
; if the verb is "turn", then we expect the "object" to be a 
; predicative adjective; possibly a prepositional object.  If the verb
; is "express", then we expect the "adverb" to be an emotion-adjective
; (sad, happy, etc.)
;
; This rule combines two checks:
; 1) Do the words have groundings that we know of?
; 2) Can the groundings be combined in a syntactically valid way?
;
; Step (2) is important, because we have to distinguish "look left"
; from "look sad" -- in the first case, the grounding of "look"
; requires a object; in the second case, "look" is grounded in a
; different way, and can only take names of facial expressions.
;
(define (obj-semantics-template VERB-GND-DECL OBJ-GND-DECL ACTION)
	(BindLink
		(VariableList
			(var-decl "$verb" "WordNode")
			(var-decl "$object" "WordNode")
			VERB-GND-DECL
			OBJ-GND-DECL
			(var-decl "$ground-verb-type" "ConceptNode")
			(var-decl "$ground-obj-type" "ConceptNode")
			(var-decl "$linkage" "PredicateNode")
		)
		(AndLink
			; The simplified pseudo-English sentence we are anayzing.
			(StateLink current-imperative
				(ActionLink
					(Variable "$verb")
					(ListLink (Variable "$object"))))

			; Candidate groundings for the words in the sentence.
			(ReferenceLink (Variable "$verb") (Variable "$verb-ground"))
			(ReferenceLink (Variable "$object") (Variable "$obj-ground"))

			; Types (kinds) of the groundings
			(InheritanceLink (Variable "$verb-ground")
				(VariableNode "$ground-verb-type"))
			(InheritanceLink (Variable "$obj-ground")
				(VariableNode "$ground-obj-type"))

			; Allowed syntactic structure of the groundings.
			(EvaluationLink
				(VariableNode "$linkage")
				(ListLink
					(VariableNode "$ground-verb-type")
					(VariableNode "$ground-obj-type")))
		)

		ACTION
))

; See description above.
(define obj-semantics-rule-1
	(obj-semantics-template
		(var-decl "$verb-ground" "GroundedPredicateNode") ; VERB-GND-DECL
		(var-decl "$obj-ground"  "DefinedSchemaNode")      ; OBJ-GND-DECL

		; We only "suggest" this as one possible action.  A later stage
		; picks the most likely action, based on some semantic liklihood
		; analysis... or soemthing like that.  Thus, we use a ListLink
		; here, not a StateLink, since the ListLink allows multiple
		; suggestions to be made.
		(ListLink current-action                         ; ACTION
			(EvaluationLink
				(Variable "$verb-ground")
				(Variable "$obj-ground")))
	))

; Like above, but for the model.
(define obj-semantic-model-rule-1
	(obj-semantics-template
		(var-decl "$verb-ground" "AnchorNode")  ; VERB-GND-DECL
		(var-decl "$obj-ground"  "ConceptNode")  ; OBJ-GND-DECL

		(StateLink
			(Variable "$verb-ground")
			(Variable "$obj-ground"))
	))

;--------------------------------------------------------------------
; Action schema
; This is wrong, but a hack for now.

(define action-rule-1
	(BindLink
		(VariableList
			(TypedVariable
				(Variable "$action")
				(Signature
					(EvaluationLink
						(Type "GroundedPredicateNode")
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
(define (imperative-process imp)
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
	(cog-bind turn-rule-3)
	(cog-bind turn-rule-4)
	(cog-bind smile-rule)
	(cog-bind frown-rule)
	(cog-bind recoil-rule)
	(cog-bind show-rule-1)
	(cog-bind show-rule-2)

	; Apply semantics-rule-1 -- if the current-imperative
	; anchor is a word we understand in a physical grounded
	; sense, then attach that sense to the current-action anchor.
	(cog-bind obj-semantics-rule-1)
(display
	(cog-bind obj-semantic-model-rule-1)
)

	; Perform the action, and print a reply.
	(let* ((act-do-do (cog-bind action-rule-1))
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
			(cog-delete-recursive (ListLink current-action x)))
				action-list)

		; XXX replace this by AIML or something.
		(if (eq? '() action-list)
			(display "I don't know how to do that.\n"))
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
