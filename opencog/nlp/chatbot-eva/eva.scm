;
; eva.scm
;
; Hacky scaffolding for talking with Hanson Robotics Eva.

;--------------------------------------------------------------------
(use-modules (opencog) (opencog nlp) (opencog query) (opencog exec))
(use-modules (opencog nlp fuzzy))
(load "../relex2logic/rule-utils.scm")

; Must load the rulebase before running eva; see bug
; https://github.com/opencog/opencog/issues/2021 for details
; XXX fixme -- we should not need to load either relex2logic or
; the rules right here, since the code in this module does not depend
; directly on thes.
(use-modules (opencog nlp relex2logic))
(load-r2l-rulebase)


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

(define (imperative-direction-rule-template VERB-WORD DECL LINKS)
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			(var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			DECL
			(var-decl "$direct-inst" "WordInstanceNode")
			(var-decl "$direction" "WordNode")
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
			(word-lemma "$direct-inst" "$direction")
		)
		(State current-imperative
			(ActionLink
				VERB-WORD
				(ListLink (Variable "$direction"))
		))
	)
)

; Re-implementation of look-rule-1 and 2, using the shorter template.
(define look-rule-1
	(imperative-direction-rule-template
		(WordNode "look")  ; VERB-WORD
		'()                ; DECL
		(ChoiceLink        ; LINKS
			(lg-link "MVa" "$verb-inst" "$direct-inst")
			(lg-link "Pa" "$verb-inst" "$direct-inst"))
	))

(define look-rule-2
	(imperative-direction-rule-template
		(WordNode "look")                           ; VERB-WORD
		(var-decl "$prep-inst" "WordInstanceNode")  ; DECL
		(list ; turn --MVp-> to --Ju-> direction    ; LINKS
			(lg-link "MVp" "$verb-inst" "$prep-inst")
			(ChoiceLink
				(lg-link "Js" "$prep-inst" "$direct-inst")
				(lg-link "Ju" "$prep-inst" "$direct-inst"))
		)
	))

; Same as look-rule-1 and look-rule-2 but with verb "turn"
(define turn-rule-3
	(imperative-direction-rule-template
		(WordNode "turn")  ; VERB-WORD
		'()                ; DECL
		(ChoiceLink        ; LINKS
			(lg-link "MVa" "$verb-inst" "$direct-inst")
			(lg-link "Pa" "$verb-inst" "$direct-inst"))
	))

(define turn-rule-4
	(imperative-direction-rule-template
		(WordNode "turn")                           ; VERB-WORD
		(var-decl "$prep-inst" "WordInstanceNode")  ; DECL
		(list ; turn --MVp-> to --Ju-> direction    ; LINKS
			(lg-link "MVp" "$verb-inst" "$prep-inst")
			(ChoiceLink
				(lg-link "Js" "$prep-inst" "$direct-inst")
				(lg-link "Ju" "$prep-inst" "$direct-inst"))
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

; Template for adjectival expressions: "Look surprised!" "Look sad!"
(define (imperative-express-adj-template ADJ-WORD)
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
			(LemmaLink (VariableNode "$verb-inst") (WordNode "look"))
		)
		(State current-imperative
			(ActionLink
				(WordNode "express") ;; SchemaNode, the verb.
				(ListLink ADJ-WORD)
		))
	)
)

(define look-happy-rule
	(imperative-express-adj-template (WordNode "happy")))
(define look-sad-rule
	(imperative-express-adj-template (WordNode "sad")))

;--------------------------------------------------------------------
; Global semantic knowledge
; See farther down below; we build a ReferenceLink attaching
; specific parsed sentences to specific actions.

(define (get-interp-node sent-node)
"
  Given a sentence, get the likliest interpretation node for it.
  At this time, it simply returns the very first interpretation.
  Yes, this is a quick hack, needs fixing. XXX FIXME.
"
	(define parse (car (cog-chase-link 'ParseLink 'ParseNode sent-node)))
	(car (cog-chase-link 'InterpretationLink 'InterpretationNode parse)))

(define (get-interp-of-r2l r2l-set-list)
"
  Given a ListLink of r2l-sets, pick out the InterpetationNode from
  each, and return those (as a list).

XXX this may be junk/obsolete, the format of r2l-sets seems to have
changed recently.  I'm confused. Current structure seems to be this:

(ReferenceLink (InterpretationNode \"sentence@f2b..\") (SetLink ...))

but this is not what the code below looks for...
"
	; find-interp takes a single SetLink
	(define (find-interp r2l-set)
		; find-inh returns #f if inh-link is not an InheritanceLink
		; It also returns #f if it is an InheritanceLink, but
		; its first member is not an InterpretationNode
		(define (find-inh inh-link)
			(if (eq? (cog-type inh-link) 'InheritanceLink)
				(eq? 'InterpretationNode
					(cog-type (car (cog-outgoing-set inh-link))))
				#f
			)
		)

		; The find returns (should return) a single InheritanceLink
		; and the first member should be the desired InterpretationNode
		(car (cog-outgoing-set
			(find find-inh (cog-outgoing-set r2l-set))))
	)

	(map find-interp (cog-outgoing-set r2l-set-list))
)

;--------------------------------------------------------------------
; Global semantic interpretation

(define neutral-gaze
	(ListLink (Number 0) (Number 0) (Number 0)))

; Global state for the current look-at point
; This state records the direction that Eva is looking at,
; right now. This can be queried with appropriate questins
; although, not yet ... XXX TODO.
(StateLink (AnchorNode "head-pointing direction") neutral-gaze)
(StateLink (AnchorNode "gaze direction") neutral-gaze)

; Global knowledge about spatial directions.  The coordinate system
; is specific to the HR robot head.  Distance in meters, the origin
; of the system is behind the eyes, middle of head.  "forward" is the
; direction the chest is facing.
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
; Emotional expression semantics (groundings)
(DefineLink
	(DefinedSchema "happy")
	(ListLink
		(Concept "happy")    ; blender animation name.
		(Number 4)           ; duration, seconds
		(Number 0.3)         ; intensity
	))

(DefineLink
	(DefinedSchema "sad")
	(ListLink
		(Concept "sad")      ; blender animation name.
		(Number 4)           ; duration, seconds
		(Number 0.3)         ; intensity
	))

;--------------------------------------------------------------------
; Global knowledge about word-meaning.
; In this case, specific words have very concrete associations
; with physical directions.

(ReferenceLink (WordNode "up") (DefinedSchema "upwards"))
(ReferenceLink (WordNode "down") (DefinedSchema "downwards"))
(ReferenceLink (WordNode "right") (DefinedSchema "rightwards"))
(ReferenceLink (WordNode "left") (DefinedSchema "leftwards"))

; Global knowledge about imperative verbs.
(ReferenceLink (WordNode "look") (GroundedPredicate "py:gaze_at_point"))
(ReferenceLink (WordNode "turn") (GroundedPredicate "py:look_at_point"))

; Currently supported facial animations on the Eva blender model:
; happy, comprehending, engaged, bored, irratated
; sad, confused, recoil, surprised
(ReferenceLink (WordNode "express") (GroundedPredicate "py:do_emotion"))
(ReferenceLink (WordNode "smile") (DefinedSchema "happy"))
(ReferenceLink (WordNode "frown") (DefinedSchema "sad"))
(ReferenceLink (WordNode "happy") (DefinedSchema "happy"))
(ReferenceLink (WordNode "sad")   (DefinedSchema "sad"))

;--------------------------------------------------------------------
; Semantic disambiguation.
; See if we know the meaning of utterances.
; These are rules to be applied to the current state: if the
; current word/utterance has an explicit concrete grounding, then
; construct the grounded equivalent.

; look-semantics-rule-1 -- if the current imperative contains a verb
; that we know, and a direction word, then suggest a grounded action.
;
(define look-semantics-rule-1
	(BindLink
		(VariableList
			(var-decl "$verb" "WordNode")
			(var-decl "$direction" "WordNode")
			(var-decl "$verb-ground" "GroundedPredicateNode")
			(var-decl "$dir-ground" "DefinedSchemaNode")
		)
		(AndLink
			(StateLink current-imperative
				(ActionLink
					(Variable "$verb")
					(ListLink (Variable "$direction"))))
			(ReferenceLink (Variable "$direction") (Variable "$dir-ground"))
			(ReferenceLink (Variable "$verb") (Variable "$verb-ground"))
		)

		; We only "suggest" this as one possible action.  A later stage
		; picks the most likely action, based on some semantic liklihood
		; analysis... or soemthing like that.  Thus, we use a ListLink
		; here, not a StateLink, since the ListLink allows multiple
		; suggestions to be made.
		(ListLink current-action
			(EvaluationLink
				(Variable "$verb-ground")
				(Variable "$dir-ground")))
))

; These are English-language sentences that I (Eva) understand.
; XXX These are not being used right now; these are meant to be
; fuzzy-matched, in a newer/different design,.... which is
; maybe abandoned right now???
(define known-directives
	(list
		(get-interp-node (car (nlp-parse "look left")))
		(get-interp-node (car (nlp-parse "look right")))
		(get-interp-node (car (nlp-parse "look up")))
		(get-interp-node (car (nlp-parse "look down")))
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

; First quick stove-pipe hack to perform an action.
(define (imperative-process imp)
"
  Process imperative IMP, which should be a SentenceNode.
"
	; Make the current sentence visible to everyone.
	(StateLink current-sentence imp)

	; apply rule-1 -- if the sentence is a command to look,
	; this will find the WordNode direction and glue it onto
	; the current-imperative anchor.
	(cog-bind look-rule-1)
	(cog-bind look-rule-2)
	(cog-bind turn-rule-3)
	(cog-bind turn-rule-4)
	(cog-bind smile-rule)
	(cog-bind frown-rule)

	; Apply semantics-rule-1 -- if the current-imperaitve
	; anchor is a word we understand in a physical grounded
	; sense, then attach that sense to the current-action anchor.
	(cog-bind look-semantics-rule-1)

	; Perform the action, and print a reply.
	(let* ((act-do-do (cog-bind action-rule-1))
			(action-list (cog-outgoing-set act-do-do))
		)
		(display act-do-do) (newline)
		(for-each cog-evaluate! action-list)

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

(define (imperative-process-v2 imp)
"
  Process imperative IMP, which should be a SentenceNode.
"

	; Get the r2l-set of the sentence
	(define r2l-set (get-r2l-set-of-sent imp))

	; Get the sentences that are similar to it.
	(define fzset (cog-fuzzy-match r2l-set))

	; Get the InterpretationNode's out of that set.
	(define interp (car (get-interp-of-r2l fzset)))

	; See if it is an interpretation that we know
	(define known (find (lambda (inp) (eq? interp inp)) known-directives))

	(if (eq? #f known)
		(display "I don't know how to do that.\n")
	)

	known
)

;--------------------------------------------------------------------
