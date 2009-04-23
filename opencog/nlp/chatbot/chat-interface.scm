;
; chat-interface.scm
;
; Simple scheme interface to glue together the chat-bot to the cog-server.
; 
; Linas Vepstas April 2009
;

(use-modules (ice-9 rdelim))

; -----------------------------------------------------------------------
; relex-parse -- send text to RelEx parser, load the resulting opencog atoms
;
; This routine takes plain-text input (in english), and sends it off 
; to a running instance of the RelEx parser, which should be listening 
; on port 4444. The parser will return a set of atoms, and these are
; then loaded into this opencog instance. This routine returns a list
; of atoms to a list of SentenceNode's that were generated as a result
; of the parse.
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
	(let ((new-sent-anchor (ConceptNode "# New Parsed Sentence"))
			(new-sent (cog-chase-link 'ListLink 'SentenceNode new-sent-anchor))
		)

		; Delete the ListLink's that connect the anchor to the new sentences
		(for-each (lambda (x) (cog-delete x)) (cog-incoming-set new-sent-anchor))

		; return the list of newly parsed sentences
		new-sent
	)
)

; -----------------------------------------------------------------------
; say-id-english -- process user input from chatbot.
; args are: user nick from the IRC channel, and the text that the user entered.
;
; Right now, this just echoes the input.
;
(define (say-id-english nick txt)
	(display "Hello ")
	(display nick)
	(display ", you said: ")
	(display txt)
	(newline)
	(display "My answer is:\n")

	; Parse the input, send it to the question processor
	(cog-ad-hoc "question" (car (relex-parse txt)))
)

