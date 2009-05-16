scm
;
; triples-pipeline.scm
;
; Triples processing pipeline. The code here looks for new parses
; attached to the (AnchorNode "# APPLY TRIPLE RULES"), and applies
; the triples processing code to each parse found there. It then
; attaches the newly found list of triples to (AnchorNode 
; "# RESULT TRIPLES")
; 
; Copyright (C) 2009, Linas Vepstas <linasvepstas@gmail.com>
;
; -----------------------------------------------------------------------
; Semantic triples processing code.
;
; The ready-for-triples-anchor is an anchor node at which sentences may
; be queued up for triples processing.  Sentences that are linked to 
; this node will eventually have triples built from them.
(define ready-for-triples-anchor (AnchorNode "# APPLY TRIPLE RULES" (stv 1 1)))

; attach-sents-for-triple-processing -- 
; Attach a list of sentences to the input triple processing anchor
; Here, sent-list should be a list of SentenceNodes
; This is slightly tricky, because the triples anchor is expecting
; not SentenceNodes, but ParseNodes.  So for each sentence, we have
; to get the parses, and attach those.
;
; return value is undefined
;
(define (attach-sents-for-triple-processing sent-list)

	;; Attach all parses of a sentence to the anchor.
	(define (attach-parses sent)
		;; Get list of parses for the sentence.
		(define (get-parses sent)
			(cog-chase-link 'ParseLink 'ParseNode sent)
		)
		;; Attach all parses of the sentence to the anchor.
		;; This must have a true/confident TV so that the pattern
		;; matcher will find and use this link.
		(for-each (lambda (x) (ListLink ready-for-triples-anchor x (stv 1 1)))
			(get-parses sent)
		)
	)
	;; Attach all parses of all sentences to the anchor.
	(for-each attach-parses sent-list)
)

; Dettach sentences that were waiting for triples processing
; However, dettach only those that don't have a VariableNode,
; since some of the ImplicationLinks will be looking for an
; attachment.
(define (dettach-sents-from-triple-anchor)
	(define (remove-anch l)
		(if (eq? 'VariableNode (cog-type (cadr (cog-outgoing-set l))))
			#f
			(cog-delete l)
		)
	)

	(for-each remove-anch (cog-incoming-set ready-for-triples-anchor))
)

; -----------------------------------------------------------------------
; The result-triples-anchor anchors the results of triples processing.
(define result-triples-anchor (AnchorNode "# RESULT TRIPLES" (stv 1 1)))

; create-triples -- extract semantic triples from RelEx dependency
; parses, using the code in the nlp/triples directory.
(define (create-triples)

	(define (attach-triples triple-list)
		;; Attach all of the recently created triples to the anchor.
		;; This must have a true/confident TV so that the pattern
		;; matcher will find and use this link.
		(for-each (lambda (x) (ListLink result-triples-anchor x (stv 1 1)))
			(cog-outgoing-set triple-list) 
		)
	)

	; First, create all of the preposition phrases that we'll need.
	(for-each
		(lambda (rule)
			(cog-ad-hoc "do-implication" rule)
		)
		prep-rule-list ; this list defined by the /triples/prep-rules.txt file
	)

	; Now, create the actual triples
	(for-each
		(lambda (rule)
			(attach-triples (cog-ad-hoc "do-implication" rule))
		)
		frame-rule-list ; this list defined by the /triples/rules.txt file
	)
)

; -----------------------------------------------------------------------
; get-new-triples -- Return a list of semantic triples that were created.
;
(define (get-new-triples)
	(cog-chase-link 'ListLink 'EvaluationLink result-triples-anchor)
)

; delete-result-triple-links -- delete links to result triples anchor.
;
(define (delete-result-triple-links)
	(for-each (lambda (x) (cog-delete x))
		(cog-incoming-set result-triples-anchor)
	)
)

; -----------------------------------------------------------------------
.
exit
