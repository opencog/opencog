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

(define (whassup) 
	"Hey there dog"
)

; -----------------------------------------------------------------------
;
; Define a super-dooper cheesy way of getting the answer to the question
; Right now, its looks for Nodes attached, via ListLink, to an 
; AnchorNode called "# QUERY SOLUTION". This is of course very wrong,
; and is just a placeholder for now.
; Anyway, three different kinds of things can be found:
; WordNodes, almost always "yes", in answer to a yes/no question.
; WordInstanceNodes, in answer to simple pattern matching
; SemeNodes, in answer to triples matching.
; Handle each of these.
; 
(define query-soln-anchor (AnchorNode "# QUERY SOLUTION"))
(define (chat-get-simple-answer)
	(define (do-one-answer answ)
		(let ((tipo (cog-type answ)))
			(cond 
				((eq? tipo 'WordNode) answ)
				((eq? tipo 'WordInstanceNode) (word-inst-get-lemma answ))
				((eq? tipo 'SemeNode) 
					; the lemma-link for a seme might still be sitting on disk!
					(load-referers answ)
					(word-inst-get-lemma answ)
				)
				(else '())
			)
		)
	)
	(map 
		(lambda (x) (do-one-answer (cadr (cog-outgoing-set x)))) 
		(cog-incoming-set query-soln-anchor)
	)
)
(define (chat-delete-simple-answer)
	(for-each (lambda (x) (cog-delete x)) 
		(cog-incoming-set query-soln-anchor)
	)
)

; -----------------------------------------------------------------------
; print stuff ...
;
(define (chat-prt-soln soln-list)
	(define (do-prt-soln soln-list)
		;; display *all* items in the list.
		(define (show-item wlist)
			(if (not (null? wlist))
				(let ((wrd (car wlist)))
					(if (null? wrd)
						(display "(null)")
						(display (cog-name wrd))
					)
					(display " ")
					(show-item (cdr wlist))
				)
			)
		)
		(display "The answer to your question is: ")
		(show-item soln-list)
	)

	(if (not (null? soln-list))
		(do-prt-soln soln-list)
	)
)

; -----------------------------------------------------------------------
; say-id-english -- process user input from chatbot.
; args are: user nick from the IRC channel, and the text that the user 
; entered.
;
; IMPORTANT NOTE: To get semi-interactive I/O while potentially lengthly
; processing is going on, the processing as been split into stages,
; designed so that each stage returns from the scheme interpreter,
; with some output for the chatbot.  The next stage of processing can
; then be continued by writing ":scm hish\r (sheme code)\n" to the 
; chat processor.
;
; This interactive design is not very pretty, and it would be better
; to come up with some sort of multi-threaded design, with one of the 
; threads listening on the scheme port.  FIXME XXX This should be fixed,
; because the current approach has *many* problems. However, the fix is
; hard, sowe just punt for now.
;
(define (say-id-english nick txt)

	; Declare some state variables for the imperative style to follow
	(define sents '())
	(define is-question #f)

	(display "Hello ")
	(display nick)
	(display ", parsing ...\n")

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
			(chat-prt-soln (chat-get-simple-answer))
		)
		(let ()
			(display nick)
			(display ", you made a statement: ")
			(display txt)
			(newline)
		)
	)

	; Invoke the next step of processing.
	(display ":scm hush\r (say-part-2 ")
	(display is-question)
	(display ")\n")
	(fflush)
	""
)

(define (say-part-2 is-question)

	; Run the triples processing.
	(attach-sents-for-triple-processing (get-new-parsed-sentences))
	(create-triples)
	(dettach-sents-from-triple-anchor)

	; If a question was asked, and the previous attempt to answer the
	; question failed, try again with pattern matching on the triples.
	(if (and is-question (null? (chat-get-simple-answer)))
		(let ((trips (get-new-triples)))
			(display "There was no simple answer; attempting triples search\n")

			; First, pull in any semes that might be related...
			(fetch-related-semes trips) 

			; Grab just the first triple for now ... and try to 
			; pattern-match it.
			(if (not (null? trips))
				(cog-ad-hoc "triple-question" (car (get-new-triples)))
			)
			(let ((answer-list (chat-get-simple-answer)))
				(if (not (null? answer-list))
					(chat-prt-soln answer-list)
					(display "No answer was found to your question.")
				)
			)
		)
	)

	; Delete list of triples, so they don't interfere with the next question.
	(delete-result-triple-links)

	; Delete  the list of solutions, so that we don't accidentally
	; replay it when the next question is asked.
	(chat-delete-simple-answer)

	; cleanup -- these sentences are not new any more
	(delete-new-parsed-sent-links)
	""
	; ":scm hush\r (whassup)"
)
