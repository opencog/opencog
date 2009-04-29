;
; chat-interface.scm
;
; Simple scheme interface to glue together the chat-bot to the cog-server.
; 
; Linas Vepstas April 2009
;

(use-modules (ice-9 rdelim))  ; for the system call

;; Hack to flush IO except this hack doesn't work :-(
;; doesn't work because of how the SchemEval.cc handles ports ... 
(define (fflush) (force-output (car  (fdes->ports 1))))

; -----------------------------------------------------------------------
; global vars:
; new-sent anchor points at the node to which all new sentences are connected
;
(define new-parsed-sent-anchor (AnchorNode "# New Parsed Sentence" (stv 1 1)))

; Return the list of SentenceNodes that are attached to the 
; freshly-parsed anchor.  This list will be non-empty if relex-parse
; has been recently run. This list can be emptied with the call
; delete-new-parsed-sent-links below.
;
(define (get-new-parsed-sentences)
	(cog-chase-link 'ListLink 'SentenceNode new-parsed-sent-anchor)
)

; delete-new-parsed-sent-links deletes the links that anchor sentences to 
; to new-parsed-sent anchor.
;
(define (delete-new-parsed-sent-links)
	(for-each (lambda (x) (cog-delete x)) (cog-incoming-set new-parsed-sent-anchor))
)

; -----------------------------------------------------------------------
; relex-parse -- send text to RelEx parser, load the resulting opencog atoms
;
; This routine takes plain-text input (in english), and sends it off 
; to a running instance of the RelEx parser, which should be listening 
; on port 4444. The parser will return a set of atoms, and these are
; then loaded into this opencog instance. After import, these are attached
; to the "new-parsed-sent-anchor" via a ListLink; the set of newly added
; sentences can be fetched with the "get-new-parsed-sentences" call.
;
(define (relex-parse plain-txt)

	; A little short routine that sends the plain-text to the
	; RelEx parser, and then loads the resulting parse into the
	; atomspace (using exec-scm-from-port to do the load)
	(define (do-sock-io sent-txt)
		(let ((s (socket PF_INET SOCK_STREAM 0)))
			(connect s AF_INET (inet-aton "127.0.0.1") 4444)
	
			(display sent-txt s)
			(display "\n" s) ; must send newline to flush socket
			(system (string-join (list "echo Info: send to parser: " sent-txt)))
			(exec-scm-from-port s)
			(shutdown s 2)
			(system (string-join (list "echo Info: close socket to parser" )))
		)
	)

	; Perform the actual processing
	(do-sock-io plain-txt)
)

; -----------------------------------------------------------------------
; Semantic triples processing code.
;
; The ready-for-triples-anchor is an anchor node at which sentences may
; be queued up for triples processing.  Sentences that are linked to 
; this node will eventually have triples built from them.
(define ready-for-triples-anchor (AnchorNode "# APPLY TRIPLE RULES" (stv 1 1)))

; copy-new-sent-to-triple-anchor -- 
; This is slightly tricky, because the triples anchor is expecting
; not SentenceNodes, but ParseNodes.  So for each sentence, we have
; to get the parses, and attach those.
;
(define (copy-new-sent-to-triple-anchor)

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
	(for-each attach-parses (get-new-parsed-sentences))
)

; Delete sentences that were wating for triples processing
(define (delete-triple-anchor-links)
	(for-each (lambda (x) (cog-delete x))
		(cog-incoming-set ready-for-triples-anchor)
	)
)

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
	(for-each
		(lambda (rule)
			(attach-triples (cog-ad-hoc "do-implication" rule))
		)
		frame-rule-list ; this list defined by the /triples/rules.txt file
	)
)

; get-new-triples -- Return a list of semantic triples that were created.
(define (get-new-triples)
	(cog-chase-link 'ListLink 'EvaluationLink result-triples-anchor)
)

; delete-result-triple-links -- delete links to result triples anchor.
(define (delete-result-triple-links)
	(for-each (lambda (x) (cog-delete x))
		(cog-incoming-set result-triples-anchor)
	)
)

; Fetch, from SQL storage, all knowledge related to the recently produced
; triples. Specifically, hunt out the WordNode's tht occur in the triples,
; and get everything we know about them (by getting everything that has 
; that word-node in it's outgoing set.)
(define (fetch-related-triples)
	; Given a handle h to some EvaluationLink, walk it down and pull
	; in any related WordNode expressions.
	(define (fetch-word h)
		(if (eq? 'WordNode (cog-type h))
			(cog-ad-hoc "fetch-incoming-set" h)
		)
		(for-each fetch-word (cog-outgoing-set h))
	)

	; Pull in related stuff for every triple that was created.
	(for-each fetch-word (get-new-triples))
)

; -----------------------------------------------------------------------
; say-id-english -- process user input from chatbot.
; args are: user nick from the IRC channel, and the text that the user entered.
;
; XXX FIXME: use of display here is no good, since nothing is written till
; processing is done.  We need to replace this by incremenntal processing
; and/or handle i/o on a distinct thread.
;
(define query-soln-anchor (AnchorNode "# QUERY SOLUTION"))
(define (say-id-english nick txt)

	; Define a super-dooper cheesy way of getting the answer to the question
	; Right now, its looks for WordNode's attached, via ListLink, to 
	; an AnchorNode called "# QUERY SOLUTION". This is of course very wrong,
	; and is just a placeholder for now.
	(define (get-simple-answer)
		(cog-chase-link 'ListLink 'WordNode query-soln-anchor)
	)
	(define (delete-simple-answer)
		(for-each (lambda (x) (cog-delete x)) (cog-incoming-set query-soln-anchor))
	)
	(define (do-prt-soln soln-list)
		;; display *all* items in the list.
		(define (show-item wlist)
			(if (not (null? wlist))
				(let ()
					(display (cog-name (car wlist)))
					(display " ")
					(show-item (cdr wlist))
				)
			)
		)
		(display "The answer to your question is: ")
		(show-item soln-list)
	)

	(define (prt-soln soln-list)
		(if (not (null? soln-list))
			(do-prt-soln soln-list)
		)
	)

	; Declare some state variables for the imperative style to follow
	(define sents '())
	(define is-question #f)

	(display "Hello ")
	(display nick)
	(display ", parsing ...\n")
	(fflush)

	; Parse the input, send it to the question processor
	(relex-parse txt)

	(set! sents (get-new-parsed-sentences))

	; Hmm. Seems like sents is never null, unless there's a 
	; programmig error in Relex.  Otherwise, it always returns 
	; something, even if the input was non-sense.
	(if (null? sents)
		(let ()
			(display nick)
			(display ", you said: \"")
			(display txt)
			(display "\" but I couldn't parse that.")
			(newline)
		)
		(set! is-question (cog-ad-hoc "question" (car sents)))
	)

	;; was a question asked?
	(if is-question 
		(let ()
			(display nick)
			(display ", you asked a question: ")
			(display txt)
			(newline)
			(prt-soln (get-simple-answer))
		)
		(let ()
			(display nick)
			(display ", you made a statement: ")
			(display txt)
			(newline)
		)
	)

	; Run the triples processing.
	(copy-new-sent-to-triple-anchor)
	(create-triples)
	(fetch-related-triples)
	(delete-triple-anchor-links)

	; If a question was asked, and  the previous attempt to answer the
	; question failed, try again with pattern matching on the triples.
	(if (and is-question (null? (get-simple-answer)))
		(let ((trips (get-new-triples)))
			; (display "There was no simple answer; attempting triples\n")
			(if (not (null? trips))
				(cog-ad-hoc "triple-question" (car (get-new-triples)))
			)
			(if (not (null? (get-simple-answer)))
				(prt-soln (get-simple-answer))
				(display "No answer was found to your question.")
			)
		)
	)

	; Delete list of triples, so they don't interfere with the next question.
	(delete-result-triple-links)

	; Delete  the list of solutions, so that we don't accidentally
	; replay it when the next question is asked.
	(delete-simple-answer)

	; cleanup -- these sentences are not new any more
	(delete-new-parsed-sent-links)
	""
)

