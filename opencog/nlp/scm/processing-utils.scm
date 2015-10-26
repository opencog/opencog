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

			; An explicit port-encoding is needed by guile-2.0.9
			(set-port-encoding! s "utf-8")

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
; This loads all the rules into the cogserver shell. This assumes that the
; cogserver is started from in-source build directory.
; TODO: This should be replaced by a module.
(define (load-r2l-rulebase)
    (load-scm-from-file "../opencog/nlp/relex2logic/loader/load-rules.scm")
    (load-scm-from-file
        "../opencog/nlp/relex2logic/loader/gen-r2l-en-rulebase.scm")
)

(define (r2l-parse sent)
"
    Runs the rules found in R2L-en-RuleBase over the RelEx output creating
    the logical representation of sentence in the atomspace. Returns `#t` on
    completion.

    This can't handle  mutliple thread execution. Thus mapping this function
    over a list of sentences even though possible is not advised.

    sent:
        - a sting containing the sentence to be parsed. An empty string is not
          accepted.
"
    (define (cog-delete-parent a-link)
        ; returns the outgoing-set of `a-link` and delete it if possible.
        ; XXX maybe this has to be part of the ure module.
        (let ((returned-list (cog-outgoing-set a-link)))
            (cog-delete a-link)
            returned-list
        )
    )

    (define (run-ure parse-node)
        ; This runs all the rules of R2L-en-RuleBase over relex parse outputs,
        ; and returns a cleaned and de-duplicated list. The relex outputs
        ; associated with 'parse-node' make the focus-set, this way, IF there
        ; are  multiple parses then each are handled independently by passing
        ; them seprately as they are likely exist in a seperate
        ; semantic-universe.
        (let* ((focus-set (SetLink (parse-get-relex-outputs parse-node)))
              (outputs (cog-delete-parent (cog-fc (SetLink) r2l-rules focus-set))))

          (while (equal? 'ListLink (cog-type (car outputs)))
              (set! outputs (fold append '() (map cog-delete-parent outputs))))
          (delete-duplicates outputs)
        )
    )

    (define (interpret parse-node)
        ; FIXME: Presently only a single interpretation is created for each
        ; parse. Multiple interpreation should be handled, when
        ; word-sense-disambiguation, anaphora-resolution and other post-processes
        ; are added to the pipeline.
        (let* ((interp-name (string-append(cog-name parse-node) "_interpretation_$X"))
               (interp-node (InterpretationNode interp-name))
               (results (run-ure parse-node)))

            ; Construct a ReferenceLink to the output
            (ReferenceLink
                interp-node
                ; The function in the SetLink returns a list of outputs that
                ; are the results of the evaluation of the relex-to-logic functions,
                ; on the relex-opencog-outputs.
                (SetLink results))

            ; Associate the interpreation with a parse, as there could be multiplie
            ; interpreations to the same parse.
            (InterpretationLink interp-node parse-node)

            (AtTimeLink
                ; FIXME: maybe opencog's internal time octime should be used. Will do for
                ; now assuming a single instance deals with a single conversation.
                (TimeNode (number->string (current-time)))
                interp-node
                (TimeDomainNode "Dialogue-System"))

        )
    #t
    )

    ; Check input to ensure that not-empty string isn't passed.
    (if (string=? sent "")
        (error "Please enter a valid sentence, not empty string"))

    ; Get the relex output for the sentence.
    (relex-parse sent)

    (map interpret (sentence-get-parses (car (get-new-parsed-sentences))))
    #t
)

; -----------------------------------------------------------------------
; nlp-parse -- Wrap the whole NLP pipeline in one function.
;
; Call the necessary functions for the full NLP pipeline.
;
(define (nlp-parse plain-text)
	; making sure  we emptied previous sentences as fail-safe
	(release-new-parsed-sents)

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

; -----------------------------------------------------------------------
; Returns the parses of strings found on each line in a file at 'filepath'.
;
; 'filepath': a string that points to the file containing the strings
;             to be parsed.
;
(define (nlp-parse-from-file filepath)
    (let*
        ((cmd-string (string-join (list "cat " filepath) ""))
        (port (open-input-pipe cmd-string))
        (line (get-line port))
        )
        (while (not (eof-object? line))
            (if (or (= (string-length line) 0)
                    (char=? #\; (string-ref (string-trim line) 0))
                )
                (set! line (get-line port))
                (if (string=? "END." line)  ; continuing the tradition of RelEx
                    (break)
                    (begin
                        (catch #t
                            (lambda ()
                                (nlp-parse line)
                            )
                            (lambda (key . parameters)
                                (begin
                                    (display "*** Unable to parse: \"")
                                    (display line)
                                    (display "\"\n")
                                )
                            )
                        )
                        (set! line (get-line port))
                    )
                )
            )
        )
        (close-pipe port)
    )
)
