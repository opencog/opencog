;
; chat-interface.scm
;
; Simple scheme interface to glue together the chat-bot to the cog-server.
; 
; Linas Vepstas April 2009
;

(use-modules (ice-9 rdelim))

; -----------------------------------------------------------------------
; do-parse -- send text to parser, load the resulting opencog atoms
;
; This routine takes plain-text input (in english), and sends it off 
; to a running instance of the RelEx parser, which should be listening 
; on port 4444. The parser will return a set of atoms, and these are
; then loaded into this opencog instance.
;
(define (do-parse txt)
	(let ((s (socket PF_INET SOCK_STREAM 0)))
		(connect s AF_INET (inet-aton "127.0.0.1") 4444)

		(display txt s)
		(display "\n" s) ; must send newline to flush socket
		(system (string-join (list "echo Info: send to parser: " txt)))
		(exec-scm-from-port s)
		(shutdown s 2)
		(system (string-join (list "echo Info: close socket to parser" )))
		""
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
	(do-parse txt)
	(cog-map-type (lambda (h) (display h) #f) 'SentenceNode)
)

