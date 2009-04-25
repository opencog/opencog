;
; chat-interface.scm
;
; Simple scheme interface to glue together the chat-bot to the cog-server.
; 
; Linas Vepstas April 2009
;

(use-modules (ice-9 rdelim))  ; for the system call

; -----------------------------------------------------------------------
; global vars:
; new-sent anchor points at the node to which all new sentences are connected
;
(define new-parsed-sent-anchor (ConceptNode "# New Parsed Sentence"))

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
; say-id-english -- process user input from chatbot.
; args are: user nick from the IRC channel, and the text that the user entered.
;
; Right now, this just echoes the input.
;
(define (say-id-english nick txt)

	; Define a super-dooper cheesy way of getting the answer to the question
	; Right now, its looks for WordNode's attached, via ListLink, to 
	; a concept node called "# QUERY SOLUTION". This is of course very wrong,
	; and is just a placeholder for now.
	(define (prt-soln)
		(let* ((query-soln-anchor (ConceptNode "# QUERY SOLUTION"))
				(soln-list (cog-chase-link 'ListLink 'WordNode query-soln-anchor))
			)
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
			(if (null? soln-list)
				(display "No answer was found to your question.")
				(let ()
					(display "The answer to your question is: ")
					(show-item soln-list)
				)
			)

			; Delete  the list of solutions, so that we don't accidentally
			; replay it when the next question is asked.
			(for-each (lambda (x) (cog-delete x)) (cog-incoming-set query-soln-anchor))
		)
	)

	(display "Hello ")
	(display nick)
	(display ", you said: ")
	(display txt)
	(newline)

	; Parse the input, send it to the question processor
	(relex-parse txt)

	(let ((is-question (cog-ad-hoc "question" (car (get-new-parsed-sentences)))))
		(if is-question 
			(prt-soln)
			(display "You made a statement")
		)
	)

	; cleanup -- these sentences are not new any more
	(delete-new-parsed-sent-links)
	""
)

