;
; imperative-rules.scm
;
; Rules for converting English-langauge imperatives into an
; intermediate, simplified form.
;
; Language processing runs in multiple steps: the rules here are the
; first step: they convert parsed (link-grammar, relex) sentences into
; a certain ad-hoc simplified intermediate form. Other parts of the
; system then compare that simplified form to the grounded knowledge,
; and extract meaningful robot actions therefrom.
;
; The intermediate form or "simplified form" that these rules generate
; is currently rather poorly defined, and subject to change or even
; complete overhaul.  The simplified form is used only because it is
; too hard to directly match parsed sentences to the grounded knowledge.
;
; The intermediate form is inspired by R2L, but differs from it, mostly
; because R2L is intended for PLN reasoning, whereas the form here is
; intended to be matched to an ontology. Perhaps someday these two forms
; should be merged. Maybe.  Right know, they are not, to avoid breaking
; stuff during development.
;
; Rule-utils needed for defintion of var-decl, etc.
(use-modules (opencog nlp relex2logic))

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

; ---------
(define (print-msg node) (display (cog-name node)) (newline) (stv 1 1))

; XXX needs to be public, so that cog-execute! can find this...
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
;
; FYI: This definition (the next 30+ lines) is just an example, and is
; not used - this definition is over-ridden, below, by a shorter, more
; abstract variant that does the same thing. This example is easier to
; understand.
(define look-rule-1
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			; (var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			(var-decl "$direct-inst" "WordInstanceNode")
			(var-decl "$direction" "WordNode")
		)
		(AndLink
			(StateLink current-sentence (Variable "$sent"))
			(parse-of-sent   "$parse" "$sent")
			; (interp-of-parse "$interp" "$parse")
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
;
; FYI: This definition (the next 30+ lines) is just an example, and is
; not used - this definition is over-ridden, below, by a shorter, more
; abstract variant that does the same thing. This example is easier to
; understand.
(define look-rule-2
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			; (var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			(var-decl "$prep-inst" "WordInstanceNode")
			(var-decl "$direct-inst" "WordInstanceNode")
			(var-decl "$direction" "WordNode")
		)
		(AndLink
			(StateLink current-sentence (Variable "$sent"))
			(parse-of-sent   "$parse" "$sent")
			; (interp-of-parse "$interp" "$parse")
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
(define (imperative-object-rule-template VERB-LIST DECL LINKS)
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			; (var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			(var-decl "$verb" "WordNode")
			DECL
			(var-decl "$obj-inst" "WordInstanceNode")
			(var-decl "$object" "WordNode")
		)
		(AndLink
			(StateLink current-sentence (Variable "$sent"))
			(parse-of-sent   "$parse" "$sent")
			; (interp-of-parse "$interp" "$parse")
			(word-in-parse   "$verb-inst" "$parse")
			(word-lemma "$verb-inst" "$verb")
			VERB-LIST
			(word-pos "$verb-inst" "verb")
			(verb-tense "$verb-inst" "imperative")
			; Specific LG linkage
			LINKS
			(word-lemma "$obj-inst" "$object")
		)
		(State current-imperative
			(ActionLink
				(Variable "$verb")
				(ListLink (Variable "$object"))
		))
	)
)

; Re-implementation of look-rule-1 and 2, using the shorter template.
; Handles sentences such as "Turn left", "Look up" and also "look happy"
; Ox link: "face me"
(define look-rule-1
	(imperative-object-rule-template
		; VERB-LIST -- a list of synonyms
		(OrLink
			(Equal (Variable "$verb") (WordNode "face"))
			(Equal (Variable "$verb") (WordNode "look"))
			(Equal (Variable "$verb") (WordNode "turn"))
		)
		'()                ; DECL
		(ChoiceLink        ; LINKS
			(lg-link "MVa" "$verb-inst" "$obj-inst")
			(lg-link "MVp" "$verb-inst" "$obj-inst")
			(lg-link "Pa" "$verb-inst" "$obj-inst")
			(lg-link "Ox" "$verb-inst" "$obj-inst"))
	))

; Handles directional sentences with "to", such as "turn to the left".
; Js and Ju links handle left, right, etc. J link handles "me"
(define look-rule-2
	(imperative-object-rule-template
		; VERB-LIST -- a list of synonyms
		(OrLink
			(Equal (Variable "$verb") (WordNode "face"))
			(Equal (Variable "$verb") (WordNode "look"))
			(Equal (Variable "$verb") (WordNode "turn"))
		)
		(var-decl "$prep-inst" "WordInstanceNode")  ; DECL
		; "turn to the left"
		(list ; turn --MVp-> to --Ju-> object       ; LINKS
			(lg-link "MVp" "$verb-inst" "$prep-inst")
			(ChoiceLink
				(lg-link "Js" "$prep-inst" "$obj-inst")
				(lg-link "Ju" "$prep-inst" "$obj-inst")
				(lg-link "J" "$prep-inst" "$obj-inst"))
		)
	))

; XXX TODO Design notes:
; Rather than hand-crafting a bunch of rules like the above, we should
; do three things:
;
; (1) implement fuzzy matching, so that anything vaguely close to the
;     desired imperative will get matched.
; (2) implement synonymous phrases, rather than listing synonymous
;     verbs explicitly.
; (3) implement automated learning of new rules, and refinement of
;     existing rules.

;--------------------------------------------------------------------

; Handle verbal expression imperatives, e.g. Smile! Frown!
(define (imperative-action-template ACTION-VERB VERB-LIST)
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			; (var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			(var-decl "$verb" "WordNode")
		)
		(AndLink
			(StateLink current-sentence (Variable "$sent"))
			(parse-of-sent   "$parse" "$sent")
			; (interp-of-parse "$interp" "$parse")
			(word-in-parse   "$verb-inst" "$parse")
			(word-pos "$verb-inst" "verb")
			(verb-tense "$verb-inst" "imperative")
			(word-lemma "$verb-inst" "$verb")
			VERB-LIST
		)
		(State current-imperative
			(ActionLink
				ACTION-VERB
				(ListLink (Variable "$verb"))
		))
	)
)

(define single-word-express-rule
	(imperative-action-template
		(WordNode "express-action") ;; SchemaNode, the verb.
		(OrLink
			(Equal (Variable "$verb") (WordNode "frown"))
			(Equal (Variable "$verb") (WordNode "recoil"))
			(Equal (Variable "$verb") (WordNode "smile"))
		)))

(define single-word-gesture-rule
	(imperative-action-template
		(WordNode "gesture-action") ;; converted to a SchemaNode, the verb.
		(OrLink
			(Equal (Variable "$verb") (WordNode "blink"))
			(Equal (Variable "$verb") (WordNode "nod"))
			(Equal (Variable "$verb") (WordNode "shake"))
			(Equal (Variable "$verb") (WordNode "yawn"))
		)))

; A bunch of synonymous verbs for adjectival commands: "be happy",
; "act sad", "look afraid", etc.
(define show-rule-1
	(imperative-object-rule-template
		; VERB-LIST
		(OrLink
			(Equal (Variable "$verb") (WordNode "act"))
			(Equal (Variable "$verb") (WordNode "be"))
			(Equal (Variable "$verb") (WordNode "look"))
			(Equal (Variable "$verb") (WordNode "play"))
		)
		'()                ; DECL
		(list              ; LINKS
			(ChoiceLink
				(lg-link "MVa" "$verb-inst" "$obj-inst")
				(lg-link "MVp" "$verb-inst" "$obj-inst")
				(lg-link "Pa" "$verb-inst" "$obj-inst"))
			; Without this constraint, this rule will clobber
			; the "look to the left" command. The prep "to" will
			; be taken as an object, giving nonsense.
			(word-pos "$obj-inst" "adjective")
		)
	))

; Direct-object imperatives: "feign happiness", "mimic fear",
; "portray confusion", etc. This is nothing more than a list of
; synonyms.
(define show-rule-2
	(imperative-object-rule-template
		(OrLink
			(Equal (Variable "$verb") (WordNode "dramatize"))
			(Equal (Variable "$verb") (WordNode "emote"))
			(Equal (Variable "$verb") (WordNode "enact"))
			(Equal (Variable "$verb") (WordNode "express"))
			(Equal (Variable "$verb") (WordNode "feign"))
			(Equal (Variable "$verb") (WordNode "impersonate"))
			(Equal (Variable "$verb") (WordNode "mime"))
			(Equal (Variable "$verb") (WordNode "mimic"))
			(Equal (Variable "$verb") (WordNode "portray"))
			(Equal (Variable "$verb") (WordNode "pretend"))
			(Equal (Variable "$verb") (WordNode "show"))
		)
		'()                       ; DECL
		(lg-link "Ou" "$verb-inst" "$obj-inst") ; LINKS
	))

;--------------------------------------------------------------------
; The Wholeshow framework
; To go into different "demo modes" by changing the weights of
; various psi-controlled-rules

(define (demo-rule-template VERB-LIST LINKS)
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			(var-decl "$verb" "WordNode")
			(var-decl "$obj-inst" "WordInstanceNode")
			(var-decl "$object" "WordNode")
		)
		(AndLink
			(StateLink current-sentence (Variable "$sent"))
			(parse-of-sent   "$parse" "$sent")
			(word-in-parse "$verb-inst" "$parse")
			(word-in-parse "$obj-inst" "$parse")
			(word-lemma "$verb-inst" "$verb")
			(word-lemma "$obj-inst" "$object")
			(word-pos "$verb-inst" "verb")
			VERB-LIST
			LINKS
		)
		(State current-imperative
			(ActionLink
				(Variable "$verb")
				(ListLink (Variable "$object"))
		))
	)
)

; Go into different demo modes, for example
; "let us show reasoning", "show saliency tracking" etc
(define demo-rule
	(demo-rule-template
		(Equal (Variable "$verb") (Word "show"))
		(lg-link "Os" "$verb-inst" "$obj-inst")
	)
)

;--------------------------------------------------------------------
