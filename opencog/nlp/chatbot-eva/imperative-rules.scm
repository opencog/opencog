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
; The intermediate form or "simplificed form" that these rules generate
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
