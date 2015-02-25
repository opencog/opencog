;
; processing-utils.scm
;
; Utilities for applying different processing steps to input sentences.
; These include getting a list of recently parsed sentences, and a 
; utility to send raw input text to the RelEx parse server, with the 
; resulting parse inserted into the cogserver atomspace.
; 
; Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
; Copyright (c) 2015 OpenCog Foundation
;

(use-modules (ice-9 regex))

; -----------------------------------------------------------------------
; Release items attached to the named anchor
;
(define (release-from-anchor anchor)
   (for-each (lambda (x) (cog-purge x))
      (cog-incoming-set anchor)
   )
)

; Return the list of SentenceNodes that are attached to the 
; freshly-parsed anchor.  This list will be non-empty if relex-parse
; has been recently run. This list can be emptied with the call
; release-new-parsed-sent below.
;
(define (get-new-parsed-sentences)
	(cog-chase-link 'ListLink 'SentenceNode (AnchorNode "# New Parsed Sentence"))
)

; release-new-parsed-sents deletes the links that anchor sentences to 
; to new-parsed-sent anchor.
;
(define (release-new-parsed-sents)
	(release-from-anchor (AnchorNode "# New Parsed Sentence"))
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
; This version will not wrap R2L atoms inside ReferenceLink.  Using
; (r2l-parse ...) is preferred.
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
	(if (string=? plain-txt "")
		(display "Please enter a valid sentence.")
		(do-sock-io plain-txt)
	)
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

; -----------------------------------------------------------------------
; r2l-parse -- A copy of relex-parse funtion modified for R2L purposes.
;
; This version will assume R2L atoms are emitted by the RelEx server, and
; wrap the R2L atoms inside a ReferenceLink with an InterpretationNode.
;
(define (r2l-parse plain-txt)
	; ---------------------------------------------------------------------
	; Patterns used for spliting the string passed from relex server
	(define pattern1 "; ##### END OF A PARSE #####")
	(define pattern2 "; ##### START OF R2L #####")

	; ---------------------------------------------------------------------
	; Splits a string into substring delimited by the a pattern and returns a list
	; of the substrings.
	; TODO: make tail recursive, for efficency
	(define (split-string a-pattern a-string)
		(let ((a-match (string-match a-pattern a-string)))
			(if (not a-match)
				(list a-string)
				(append (list (match:prefix a-match))
					(split-string a-pattern (match:suffix a-match)))
			)
		)
	)

	; ---------------------------------------------------------------------
	; returns the name of a ParseNode if the string has a ParseNode entry.
	; returns "sentence@dbce9f0d-8b8a-4ea7-a8c8-d69abf05f810_parse_1" from the string
	; "84590fb2e46c (ParseNode \"sentence@dbce9f0d-8b8a-4ea7-a8c8-d69abf05f810_parse_1\""
	(define (parse-str a-string)
		(substring (match:substring
			(string-match "ParseNode \"sentence@[[:alnum:]_-]*" a-string)) 11)
	)

	; ---------------------------------------------------------------------
	; Evaluate the string and returns a list with SetLink of the initial r2l rule application.
	; (ReferenceLink 
	;   (InterpretationNode "sentence@1d220-c7ace2f_parse_2_interpretation_$X")
	;   (SetLink
	;       different links that are a result of r2l rule-functions being applied
	;   )
	; )
	;
	; A ReferenceLink is used instead of InterpretationLink so as to differentiate
	; the final word-sense-disambiguated , anaphore and cataphor resolved version from
	; the initial r2l output. The final version will have structure that is detailed
	; @ http://wiki.opencog.org/w/Linguistic_Interpretation.
	; Having the output from this function should help during garbage-removal from the
	; atomspace as well as the generation of the final set of interpretations for the
	; the sentence.
	(define (set-link a-string)
		; z-list is a list containing a set of lists, with firts elements of the
		; sub-list being a relex-opencog-output string and the second element being
		; a string of the relex-to-logic function calls that is to be applied on the
		; relex-opencog-output in the atomspace. The last sub-list is just the AnchorNode.
		(define z-list (map (lambda (x) (split-string pattern2 x)) 
		                                (split-string pattern1 a-string)))

		; helper function that prune away atoms that no longer exists from subsequent
		; rules, or atoms that are wrapped inside another link
		(define (pruner x)
			(define deref-x (cog-atom (cog-handle x)))
			; if 'deref-x' is #<Invalid handle>, than both cog-node?
			; and cog-link? will return false
			(if (and (or (cog-node? deref-x) (cog-link? deref-x))
			         (null? (cog-incoming-set x)))
				x
				#f
			)
		)

		; Given any one of the sub-lists from the z-lists it will evaluate the strings
		; elements from left to right resulting in the creation of the Atoms of the 
		; relex-to-logic pipeline.
		(define (eval-list a-list)
			(if (= 1 (length a-list))
				(begin (eval-string (list-ref a-list 0)) '())
				(begin
					(eval-string (list-ref a-list 0))
					(let* ((parse-name (parse-str (list-ref a-list 0)))
					       (parse-node (ParseNode parse-name)))
;
; XXX not really necessary at this point since we will only need the LG
; entries for the word-inst, which is emitted by the RelEx server already.
; The full LG disjuncts entries are needed only by SuReal, which already
; calls (lg-get-dict-entry ...) on its node.
;
;						; generate the LG dictionary entries for each word
;						(let ((words (parse-get-words parse-node)))
;							(map-word-instances
;								(lambda (word-inst) (map-word-node lg-get-dict-entry word-inst))
;								parse-node
;							)
;						)
						(ReferenceLink 
							(InterpretationNode (string-append parse-name "_interpretation_$X"))
							; The function in the SetLink returns a list of outputs that
							; are the results of the evaluation of the relex-to-logic functions,
							; on the relex-opencog-outputs.
							(SetLink
								(filter-map pruner
									(delete-duplicates 
										(apply append 
											(map-in-order eval-string
												(filter (lambda (x) (not (string=? "" x)))
													(split-string "\n" (list-ref a-list 1))
												)
											)
										)
									)
								)
							)
						)
						(InterpretationLink
							(InterpretationNode (string-append parse-name "_interpretation_$X"))
							parse-node
						)
					)
				)
			)
		)

		(par-map eval-list z-list)
	)

	; ---------------------------------------------------------------------
	; A helper function 
	(define (set-interpret port)
		(let ((string-read (get-string-all port)))
			(if (eof-object? string-read)
				#f
				(set-link string-read)
			)
		)
	)

	(define (do-sock-io sent-txt)
		(let ((s (socket PF_INET SOCK_STREAM 0)))
			(connect s AF_INET (inet-pton AF_INET relex-server-host) relex-server-port)

			(display sent-txt s)
			(display "\n" s)
			(system (string-join (list "echo \"Info: send to parser: " sent-txt "\"")))
			(set-interpret s)
			(system (string-join (list "echo Info: close socket to parser" )))
			(close-port s)
		)
	)

	(if (string=? plain-txt "")
		(display "Please enter a valid sentence.")
		(do-sock-io plain-txt)
	)
)

; -----------------------------------------------------------------------
; nlp-parse -- Wrap the whole NLP pipeline in one function.
; 
; Call the necessary functions for the full NLP pipeline.
;
(define (nlp-parse plain-text)
	; call the RelEx server
	(r2l-parse plain-text)
	
	(let ((sent-nodes (get-new-parsed-sentences)))
		; increment the R2L's node count value
		(parallel-map-parses
			(lambda (p)
				; The preferred algorithm is
				; (1) get all non-abstract nodes
				; (2) delete duplicates
				; (3) get the corresponding abstract nodes
				; (4) update count
				(let* ((all-nodes (append-map cog-get-all-nodes (parse-get-r2l-outputs p)))
				       ; XXX FIXME this is undercounting since each abstract node can have
				       ; multiple instances in a sentence.  Since there is no clean way
				       ; to get to the abstracted node from an instanced node yet, such
				       ; repeatition are ignored for now
				       ; XXX FIXME R2L's rule-helpers are reseting the TV to stv everytime,
				       ; that need to be removed before this code work
				       (abst-nodes (delete-duplicates (filter is-r2l-abstract? all-nodes))))
					(par-map
						(lambda (n)
							(let* ((atv (cog-tv->alist (cog-tv n)))
							       (mean (assoc-ref atv 'mean))
							       (conf (assoc-ref atv 'confidence))
							       (count (assoc-ref atv 'count))
							       ; STV will have count value as well, so checking type
							       ; to see whether we want that count value
							       (ntv
							       	(if (cog-ptv? (cog-tv n))
								 		(cog-new-ptv mean conf (+ count 1))
								 		(cog-new-ptv mean conf 1)
								 	)
							       ))
								(cog-set-tv! n ntv)
							)
						)
						abst-nodes
					)
				)
			)
			sent-nodes
		)
		(release-new-parsed-sents)
	
		; return the list of SentenceNode
		sent-nodes
	)
)

