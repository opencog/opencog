;
; model-query.scm
;
; Processing of query-questions about the robot self-model.
;
(use-modules (opencog query))
(use-modules (opencog nlp sureal))

; Rule-utils needed for defintion of var-decl, etc.
(use-modules (opencog nlp relex2logic))

(define current-sentence (AnchorNode "*-eva-current-sent-*"))
(define current-reply (AnchorNode "*-reply-*"))

;--------------------------------------------------------------------
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
;			(var-decl "$direction" "ConceptNode")
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
			(LemmaLink (VariableNode "$subj-inst") (WordNode "you"))

; XXX FIXME This is wrong; this has been replaced by the eva-model
; code in the ros-behavior-scripting tree. Unfortunately, it does
; not offer any easy way of querying.
			; (State (Anchor "*-gaze-direction-*") (Variable "$direction"))
		)
		(ListLink
			current-reply
			(SetLink
				(Evaluation (Predicate "looking") (ListLink (Concept "I")))
(VariableNode "$verb-inst")
;				(InheritanceLink
;					(SatisfyingSet (Predicate "looking")) (Variable "$direction"))
			)
		)
	)
)

;--------------------------------------------------------------------
;
; Debug utility to print the current sentence.
(define prt-sent
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			; (var-decl "$interp" "InterpretationNode")
			(var-decl "$word-inst" "WordInstanceNode")
			(var-decl "$word" "WordNode")
		)
		(AndLink
			(StateLink current-sentence (Variable "$sent"))
			(parse-of-sent   "$parse" "$sent")
			; (interp-of-parse "$interp" "$parse")
			(word-in-parse   "$word-inst" "$parse")
			(LemmaLink (Variable "$word-inst") (Variable "$word"))
		)
		(ListLink
			(Variable "$word")
		)
	)
)

(define (prt-curr-sent) (cog-bind prt-sent))

;--------------------------------------------------------------------
; XXX hack
(define face-expression-state (AnchorNode "Facial Expression State"))


; Recognize copular sentence "what are you doing?"
; This is insane overkill for the mere task of recognizing a sentence.
; The whole point of such complex analysis is to ...???
(define what-doing-rule
	(BindLink
		(VariableList
			(var-decl "$sent" "SentenceNode")
			(var-decl "$parse" "ParseNode")
			; (var-decl "$interp" "InterpretationNode")
			(var-decl "$verb-inst" "WordInstanceNode")
			(var-decl "$qvar-inst" "WordInstanceNode")
			(var-decl "$subj-inst" "WordInstanceNode")
			(var-decl "$expression" "ConceptNode")
		)
		(AndLink
			(StateLink current-sentence (Variable "$sent"))
			(parse-of-sent   "$parse" "$sent")
			; (interp-of-parse "$interp" "$parse")
			(word-in-parse   "$verb-inst" "$parse")
			(word-in-parse   "$qvar-inst" "$parse")
			(LemmaLink (VariableNode "$qvar-inst") (WordNode "what"))
			(LemmaLink (VariableNode "$verb-inst") (WordNode "do"))
			(word-pos "$verb-inst" "verb")
			(dependency "_subj" "$verb-inst" "$subj-inst")
			(LemmaLink (VariableNode "$subj-inst") (WordNode "you"))

; placeholder
			(State face-expression-state (Variable "$expression"))

		)
		(ListLink
			current-reply
			(SetLink
				(Evaluation (Predicate "doing") (ListLink (Concept "I")))
(VariableNode "$expression")
;				(InheritanceLink
;					(SatisfyingSet (Predicate "looking")) (Variable "$direction"))
			)
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
; XXX This is broken.
;
; This is where I wish I had lexical functions rather than just sureal.
; The problem here is that sureal, all by itself, is unable to convert
; "leftwards" into the synonymous but more appropriate "to the left" as
; the desired response.
;
; Also: personality and randomization: I want her to sometimes say "I am
; looking sideways".

(define (self-wh-query QUERY)
"
  Process a query about self.  Return an answer, or else nil, if
  no answer is known.  QUERY should be a SentenceNode.
"

	; Make the current sentence visible to everyone.
	(StateLink current-sentence QUERY)

	(cog-bind where-look-rule)
	; (cog-bind what-doing-rule)

(display "duuuude bar")
(display (cog-incoming-set current-reply))

	; hack fixme
; problem here is that incoming set may have more than one.
	(let* ((rep-lnk (cog-incoming-set current-reply))
			(r2l-set (cog-outgoing-atom (car rep-lnk) 1))
		)
		; Free up anything attached to the anchor.
		(map cog-delete (cog-incoming-set current-reply))
(display "duuuude wfooo")
		(display r2l-set)
		(if (equal? 0 cog-arity r2l-set)
			(list (list "Sorry I didn't understand the question.\n"))
			(let
				((string-seq (sureal r2l-set)))
				(display string-seq) (newline)
				string-seq
			)
		)
	)
)

;--------------------------------------------------------------------
