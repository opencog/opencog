;
; processing-utils.scm
;
; Utilities for applying different processing steps to input sentences.
; These include getting a list of recently parsed sentences, and a 
; utility to send raw input text to the RelEx parse server, with the 
; resulting parse inserted into the cogserver atomspace.
; 
; Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
;
; -----------------------------------------------------------------------
; Release items attached to the named anchor
;
(define (release-from-anchor anchor)
   (for-each (lambda (x) (cog-delete x))
      (cog-incoming-set anchor)
   )
)

; -----------------------------------------------------------------------
; global vars:
; new-sent anchor points at the node to which all new sentences are connected
;
(define *new-parsed-sent-anchor* (AnchorNode "# New Parsed Sentence" (stv 1 1)))

; Return the list of SentenceNodes that are attached to the 
; freshly-parsed anchor.  This list will be non-empty if relex-parse
; has been recently run. This list can be emptied with the call
; release-new-parsed-sent below.
;
(define (get-new-parsed-sentences)
	(cog-chase-link 'ListLink 'SentenceNode *new-parsed-sent-anchor*)
)

; release-new-parsed-sents deletes the links that anchor sentences to 
; to new-parsed-sent anchor.
;
(define (release-new-parsed-sents)
	(release-from-anchor *new-parsed-sent-anchor*)
)

; -----------------------------------------------------------------------
; relex-parse -- send text to RelEx parser, load the resulting opencog atoms
;
; This routine takes plain-text input (in english), and sends it off 
; to a running instance of the RelEx parser, which should be listening 
; on port 4444. The parser will return a set of atoms, and these are
; then loaded into this opencog instance. After import, these are attached
; to the "*new-parsed-sent-anchor*" via a ListLink; the set of newly added
; sentences can be fetched with the "get-new-parsed-sentences" call.
;
; The relex-server-host and port are set in config.scm, and default to
; localhost 127.0.0.1 and port 4444
;
(define (relex-parse plain-txt)

	; A little short routine that sends the plain-text to the
	; RelEx parser, and then loads the resulting parse into the
	; atomspace (using exec-scm-from-port to do the load)
	(define (do-sock-io sent-txt)
		(let ((s (socket PF_INET SOCK_STREAM 0)))
			; inet-aton is deprecated, so don't use it (as of 2013)
			; (connect s AF_INET (inet-aton relex-server-host) relex-server-port)
			(connect s AF_INET (inet-pton AF_INET relex-server-host) relex-server-port)
	
			(display sent-txt s)
			(display "\n" s) ; must send newline to flush socket
			(system (string-join (list "echo \"Info: send to parser: " sent-txt "\"")))
			(exec-scm-from-port s)
			(system (string-join (list "echo Info: close socket to parser" )))
			(close-port s)
		)
	)

	; Perform the actual processing
	(do-sock-io plain-txt)
)

; -----------------------------------------------------------------------
; get-parses-of-sents -- return parses of the sentences
; Given a list of sentences, return a list of parses of those sentences.
; That is, given a List of SentenceNode's, return a list of ParseNode's
; associated with those sentences.
;
; OPENCOG RULE: FYI this could be easily implemented as a pattern match,
; and probably should be, when processing becomes fully rule-driven.

(define (get-parses-of-sents sent-list)
	(define (get-parses sent)
		(cog-chase-link 'ParseLink 'ParseNode sent)
	)
	(concatenate! (map get-parses sent-list))
)

; -----------------------------------------------------------------------
; attach-parses-to-anchor -- given sentences, attach the parses to anchor.
; 
; Given a list of sentences i.e. a list of SentenceNodes, go through them,
; locate the ParseNodes, and attach the parse nodes to the anchor.
;
; return value is undefined (no return value).
;
; OPENCOG RULE: FYI this could be easily implemented as a pattern match,
; and probably should be, when processing becomes fully rule-driven.
;
(define (attach-parses-to-anchor sent-list anchor)

	;; Attach all parses of a sentence to the anchor.
	(define (attach-parses sent)
		;; Get list of parses for the sentence.
		(define (get-parses sent)
			(cog-chase-link 'ParseLink 'ParseNode sent)
		)
		;; Attach all parses of the sentence to the anchor.
		;; This must have a true/confident TV so that the pattern
		;; matcher will find and use this link.
		(for-each (lambda (x) (ListLink anchor x (stv 1 1)))
			(get-parses sent)
		)
	)
	;; Attach all parses of all sentences to the anchor.
	(for-each attach-parses sent-list)
)

; -----------------------------------------------------------------------
; -----------------------------------------------------------------------
