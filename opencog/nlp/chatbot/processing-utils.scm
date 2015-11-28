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

(use-modules (ice-9 regex)
             (ice-9 threads)  ; needed for par-map
             (srfi srfi-1))

; -----------------------------------------------------------------------
(define (release-from-anchor anchor)
"
  Release items attached to the named anchor
"
   (for-each (lambda (x) (cog-purge x))
      (cog-incoming-set anchor)
   )
)

(define (get-new-parsed-sentences)
"
  Return the list of SentenceNodes that are attached to the
  freshly-parsed anchor.  This list will be non-empty if relex-parse
  has been recently run. This list can be emptied with the call
  release-new-parsed-sent below.
"
	(cog-chase-link 'ListLink 'SentenceNode (AnchorNode "# New Parsed Sentence"))
)

(define (release-new-parsed-sents)
"
  release-new-parsed-sents deletes the links that anchor sentences to
  to new-parsed-sent anchor.
"
	(release-from-anchor (AnchorNode "# New Parsed Sentence"))
)

; -----------------------------------------------------------------------
(define (relex-parse plain-txt)
"
  relex-parse -- send text to RelEx parser, load the resulting opencog atoms

  This routine takes plain-text input (in English), and sends it off
  to a running instance of the RelEx parser, which should be listening
  on port 4444. The parser will return a set of atoms, which are
  then loaded into this opencog instance. After import, these are attached
  to the \"*new-parsed-sent-anchor*\" via a ListLink; the set of newly added
  sentences can be fetched with the \"get-new-parsed-sentences\" call.

  The relex-server-host and port are set in config.scm, and default to
  localhost 127.0.0.1 and port 4444

  This routine does NOT perform R2L processing.
"
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
(define (get-parses-of-sents sent-list)
"
  get-parses-of-sents -- return parses of the sentences

  Given a list of sentences, return a list of parses of those sentences.
  That is, given a List of SentenceNode's, return a list of ParseNode's
  associated with those sentences.
"
;  OPENCOG RULE: FYI this could be easily implemented as a pattern match,
;  and probably should be, when processing becomes fully rule-driven.
	(define (get-parses sent)
		(cog-chase-link 'ParseLink 'ParseNode sent)
	)
	(concatenate! (map get-parses sent-list))
)

; -----------------------------------------------------------------------
(define (attach-parses-to-anchor sent-list anchor)
"
  attach-parses-to-anchor -- given sentences, attach the parses to anchor.

  Given a list of sentences i.e. a list of SentenceNodes, go through them,
  locate the ParseNodes, and attach the parse nodes to the anchor.

  return value is undefined (no return value).
"
;  OPENCOG RULE: FYI this could be easily implemented as a pattern match,
;   and probably should be, when processing becomes fully rule-driven.

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
(define (r2l-parse sent)
"
  r2l-parse SENT -- perform relex2logic processing on sentence SENT.

  Runs the rules found in R2L-en-RuleBase over the RelEx output creating
  the logical representation of sentence in the atomspace. Returns a list
  containing SetLinks that are the r2l-interpretations for individual parses.

  This can't handle  mutliple thread execution. Thus mapping this function
  over a list of sentences even though possible is not advised.

  SENT must be a SentenceNode.
"
    (define (cog-delete-parent a-link)
        ; returns the outgoing-set of `a-link` and delete it if possible.
        ; XXX maybe this has to be part of the ure module.
        (let ((returned-list (cog-outgoing-set a-link)))
            (cog-delete a-link)
            returned-list
        )
    )

    (define (run-fc parse-node interp-link)
        ; This runs all the rules of R2L-en-RuleBase over relex parse outputs,
        ; and returns a cleaned and de-duplicated list. The relex outputs
        ; associated with 'parse-node' make the focus-set, this way, IF there
        ; are  multiple parses then each are handled independently by passing
        ; them seprately as they are likely exist in a seperate
        ; semantic-universe.
        (let* ((focus-set (SetLink (parse-get-relex-outputs parse-node) interp-link))
              (outputs (cog-delete-parent (cog-fc (SetLink) r2l-rules focus-set))))

              (append-map cog-delete-parent
                  (append-map cog-delete-parent outputs))
        )
    )

    (define (interpret parse-node)
        ; FIXME: Presently only a single interpretation is created for each
        ; parse. Multiple interpreation should be handled, when
        ; word-sense-disambiguation, anaphora-resolution and other post-processes
        ; are added to the pipeline.
        (let* ((interp-name (string-append(cog-name parse-node) "_interpretation_$X"))
               (interp-node (InterpretationNode interp-name))
               ; Associate the interpreation with a parse, as there could be multiplie
               ; interpreations to the same parse.
               (interp-link (InterpretationLink interp-node parse-node))
               (pre-result
                   (remove
                       (lambda (a) (equal? (cog-type a) 'ReferenceLink))
                       (delete-duplicates (run-fc parse-node interp-link))))
               (result (SetLink pre-result)))

            ; Construct a ReferenceLink to the output
            (ReferenceLink interp-node result)

            ; Time stamp the parse
            (AtTimeLink
                ; FIXME: maybe opencog's internal time octime should be used. Will do for
                ; now assuming a single instance deals with a single conversation.
                (TimeNode (number->string (current-time)))
                interp-node
                (TimeDomainNode "Dialogue-System"))

            result
        )
    )

    (map interpret (sentence-get-parses sent))
)

; -----------------------------------------------------------------------
(define (r2l-count sent-list)
"
  r2l-count SENT -- maintain counts of R2L statistics for SENT-LIST.
"
	; Increment the R2L's node count value
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
										(cog-new-ptv mean conf 1))
								))
							(cog-set-tv! n ntv)
						)
					)
					abst-nodes
				)
			)
		)
		sent-list
	)
)

; -----------------------------------------------------------------------
(define-public (nlp-parse plain-text)
"
  nlp-parse -- Wrap most of the NLP pipeline in one function.

  Call the necessary functions for the full NLP pipeline.
"
	; Discard previous sentences, if any.
	(release-new-parsed-sents)

	; Check input to ensure that not-empty string isn't passed.
	(if (string=? plain-text "")
		(error "Please enter a valid sentence, not empty string"))

	; Call the RelEx server
	(relex-parse plain-text)

	; Perform the R2L processing.
	(r2l-parse (car (get-new-parsed-sentences)))

	; Track some counts needed by R2L.
	(r2l-count (get-new-parsed-sentences))

	(let ((sent-list (get-new-parsed-sentences)))
		(release-new-parsed-sents)
		; Return the sentence list (why ???)
		sent-list
	)
)

; -----------------------------------------------------------------------
(define (nlp-parse-from-file filepath)
"
  Returns the parses of strings found on each line in a file at 'filepath'.

  'filepath': a string that points to the file containing the strings
              to be parsed.
"
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
