;
; eva.scm
;
; Hacky scaffolding for talking with Hanson Robotics Eva.

;--------------------------------------------------------------------
(use-modules (opencog) (opencog nlp) (opencog query) (opencog exec))
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
;
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

;--------------------------------------------------------------------
; Global semantic knowledge
; See farther down below; we build a ReferenceLink attaching
; specific parsed sentences to specific actions.

(define (get-interp-node sent-node)
"
  Given a sentence, get the likliest interpretation node for it.
  Yes, this is a quick hack, needs fixing. XXX FIXME.
"
	(define parse (car (cog-chase-link 'ParseLink 'ParseNode sent-node)))
	(car (cog-chase-link 'InterpretationLink 'InterpretationNode parse)))

(define (get-interp-of-r2l r2l-set-list)
"
  Given a list of r2l-sets, pick out the InterpetationNode from each,
  and return those (as a list).
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

; These are English-language sentences that I understand.
(define known-directives
	(list
		(get-interp-node (car (nlp-parse "look left")))
		(get-interp-node (car (nlp-parse "look right")))
		(get-interp-node (car (nlp-parse "look up")))
		(get-interp-node (car (nlp-parse "look down")))))

;--------------------------------------------------------------------
; Action schema
; This is wrong, but a hack for now.

(define look-action-rule-1
	(BindLink
		(VariableList
			; (var-decl "$action" "DefinedSchemaNode")
			(var-decl "$action" "ListLink")
		)
		(AndLink
			(StateLink current-action (Variable "$action"))
		)
		(Evaluation (GroundedPredicate "py:look_at_point")
			(Variable "$action"))
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

	; Apply semantics-rule-1 -- if the current-imperaitve
	; anchor is a word we understand in a physical grounded
	; sense, then attach that sense to the current-action anchor.
	(cog-bind look-semantics-rule-1)

	; Perform the action, and print a reply.
	(let* ((act-do-do (cog-bind look-action-rule-1))
			(action-list (cog-outgoing-set act-do-do))
		)
		(display act-do-do)
		(newline)
		(for-each cog-evaluate! action-list)

		; XXX replace this by AIML or something.
		(if (eq? '() action-list)
			(display "I don't know how to do that.\n"))
	)
)

;--------------------------------------------------------------------

(define (imperative-process-v2 imp)
"
  Process imperative IMP, which should be a SentenceNode.
"

	; Get the r2l-set of the sentence
	(define r2l-set (get-r2l-set-of-sent imp))

	; Get the sentences that are similar to it.
	(define fzset (cog-fuzzy-match r2l-set 'SetLink '()))

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
