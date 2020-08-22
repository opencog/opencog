;
; model-query.scm
;
; Processing of query-questions about the robot self-model.
;
(use-modules (srfi srfi-1))
(use-modules (opencog nlp sureal))

; Rule-utils needed for defintion of var-decl, etc.
(use-modules (opencog nlp relex2logic))

; `current-reply` holds the "semantic" (aka R2L) format of the reply.
; `current-reply-words` holds the word-sequence of the reply.
(define current-sentence (AnchorNode "*-eva-current-sent-*"))
(define current-reply (AnchorNode "*-reply-*"))
; (define current-reply-words (AnchorNode "*-reply-words-*"))

(StateLink current-reply (Set))
; (StateLink current-reply-words (List))

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
; code Unfortunately, it does not offer any easy way of querying.
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

(define (prt-curr-sent) (cog-execute! prt-sent))

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
;
; XXX this should be moved to cog-utils. Also needs to be fixed
; to not detect bound variables. We already have C++ code that
; does  the right thing, here, so we should use that.
(define (cog-grounded? EXPR)
"
  cog-grounded? EXPR

  Return #f if EXPR contains a VariableNode, else return #t.
"
	(if (cog-node? EXPR)
		(not (eq? 'VariableNode (cog-type EXPR)))
		(not (find (lambda (x) (not (cog-grounded? x))) (cog-outgoing-set EXPR)))
	)
)

; Some of the things chained to the reply-anchor are parts of rules;
; we want to ignore these.  They will contain variables, in general.
(define (get-grounded-replies)
	(filter cog-grounded? (cog-incoming-set current-reply))
)

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

	; Small utility that converts rep-lnk to a string of words.
	; Uses sureal to perform the conversion. The input must be
	; of teh form that sureal can handle.
	(define (verbalize-reply rep-lnk)
		(if (eq? 0 (cog-arity rep-lnk))
			'()
			(let ((r2l-set (cog-outgoing-atom rep-lnk 1)))

(format #t "The reply is:\n~a\n" r2l-set)
				(if (eq? 0 cog-arity r2l-set)
					'()
					(let
						((string-seq (sureal r2l-set)))
						(display string-seq) (newline)
						string-seq
					)))))

	; Make the current sentence visible to everyone.
	(StateLink current-sentence QUERY)

	; (cog-execute! where-look-rule)
	(cog-execute! what-doing-rule)

(format #t  "Replies to questions:\n~a\n" (get-grounded-replies))

	; There may be more than one plausible reply. We should use
	; openpsi to pick one. XXX FIXME -- do this.
	; Alternately, this shoud probably be done with chatscript.
	(let ((reply-words (filter verbalize-reply (get-grounded-replies))))
(format #t "Reply words are: ~a\n" reply-words)
		(if (null? reply-words)
			(set! reply-words
				(list "Sorry I didn't understand the question.\n"))
		)

		; Free up anything attached to the anchor.
		(map cog-extract-recursive! (get-grounded-replies))

		reply-words
	)
)

;--------------------------------------------------------------------
