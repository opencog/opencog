;
; semantics.scm
;
; Semantic interpretation simplified utterances.
; Try to extract a grounded meaning from utterances.
;
; The linguistic processing is done in two steps.  In the first step,
; link-grammar (relex) parses are converted to a "simplified form".
; In the second step, this simplified form is compared to the ontology
; of grounded knowledge, to see if anything matches.  The result of
; such matching is a grounded action (verb, or subroutine call) and the
; associated subroutine parameters.
;
; At this time, the "simplfied form" is completely ad-hoc and poorly
; defined.  It exists only because the mapping of parsed sentences to
; grounded knowledge with only one step seems too hard; so instead we
; digest the sentence a bit, and only then try to ground the digested
; form of it.
;
; All this needs to be revamped with some kind of more general mechanism
; to fuzzy-match parsed sentences to the grounded knowledge.
;
; At this time, this file is primarily concerned with imperatives.
;
; These are rules to be applied to the current state: if the
; current word/utterance has an explicit concrete grounding, then
; construct the grounded equivalent.
;--------------------------------------------------------------------

; Rule-utils needed for defintion of var-decl, etc.
(use-modules (opencog nlp relex2logic))

(define current-action (AnchorNode "*-action-*"))
(define current-imperative (AnchorNode "*-imperative-*"))

;--------------------------------------------------------------------
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
			; The simplified pseudo-English sentence we are analyzing.
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
; Verb uses DefinedPredicateNode so that groundings are run through the
; action orchestrator.
(define obj-semantics-rule-1-ao
	(obj-semantics-template
		(var-decl "$verb-ground" "DefinedPredicateNode")  ; VERB-GND-DECL
		(var-decl "$obj-ground"  "DefinedSchemaNode")     ; OBJ-GND-DECL

		; We only "suggest" this as one possible action.  A later stage
		; picks the most likely action, based on some semantic liklihood
		; analysis... or something like that.  Thus, we use a ListLink
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
		(var-decl "$obj-ground"  "ConceptNode") ; OBJ-GND-DECL

		(StateLink                              ; ACTION
			(Variable "$verb-ground")
			(Variable "$obj-ground"))
	))

(define obj-semantic-model-rule-2
	(obj-semantics-template
		(var-decl "$verb-ground" "AnchorNode")         ; VERB-GND-DECL
		(var-decl "$obj-ground"  "DefinedSchemaNode")  ; OBJ-GND-DECL

		(StateLink                                     ; ACTION
			(Variable "$verb-ground")
			(Variable "$obj-ground"))
	))

;--------------------------------------------------------------------
