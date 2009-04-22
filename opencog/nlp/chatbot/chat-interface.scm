;
; chat-interface.scm
;
; Simple scheme interface to glue together the chat-bot to the cog-server.
; 
; Linas Vepstas April 2009
;
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
)
