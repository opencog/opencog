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

(use-modules (ice-9 popen)    ; needed for open-pipe, close-pipe
             (ice-9 threads)  ; needed for with-mutex
             (rnrs io ports)  ; needed for get-line
             (srfi srfi-1)
             (opencog)
             (opencog atom-types))

; -----------------------------------------------------------------------
; See below for the docs.
(define-public get-one-anchored-item
	(let ((mtx (make-mutex)))
		(lambda (ANCHOR)
			(with-mutex mtx
				(let ((iset (cog-incoming-by-type ANCHOR 'ListLink)))
					(if (null? iset) '()
						(let* ((lnk (car iset))
								(item (cog-get-partner lnk ANCHOR)))
							(cog-extract-recursive lnk)
							item))))))
)

(set-procedure-property! get-one-anchored-item 'documentation
"
  get-one-anchored item ANCHOR - dettach one item from the ANCHOR

  Each call to this will return one (or zero) items attached to
  the ANCHOR. It will do so quasi-atomically, in that one and only
  one caller will get the item; it is dettached from the anchor after
  this call. Its only \"quasi-atomic\", because non-scheme users (e.g.
  C++, python) will still race against this, as this uses a scheme
  mutex for protection.  Ideally, we should invent a new atom-type
  to do this, I guess .. some rainy day.
")

(define-public (release-from-anchor anchor)
"
  release-from-anchor ANCHOR

  Release all items attached to ANCHOR.
"

	; Use cog-extract-recursive, not cog-extract, because some poorly
	; written code somewhere is running a badly-scoped search pattern,
	; which results in links getting wrapped in a SetLink. So the
	; recursive-extract forcibly breaks such mal-formed usages.
	(for-each (lambda (x) (cog-extract-recursive x))
		(cog-incoming-set anchor)
	)
)

(define-public (get-new-parsed-sentences)
"
  get-new-parsed-sentences -- return newly parsed sentences.

  Return the list of SentenceNode's that are attached to the
  freshly-parsed anchor.  This list will be non-empty if relex-parse
  has been recently run. This list can be emptied with the call
  release-new-parsed-sent below.

  CAUTION: This is NOT thread-safe -- two different threads that
  are trying to get them all, and then using `release-new-parsed-sents`
  will typically step on each-other.  Use `get-one-new-sentence` instead.
"
	(cog-chase-link 'ListLink 'SentenceNode (AnchorNode "# New Parsed Sentence"))
)

(define-public (release-new-parsed-sents)
"
  release-new-parsed-sents deletes the links that anchor sentences to
  to new-parsed-sent anchor.
"
	(release-from-anchor (AnchorNode "# New Parsed Sentence"))
)

(define-public (get-one-new-sentence)
"
  get-one-new-sentence - get one recently parsed sentence, uniquely.

  Each call to this will return one (or zero) sentences attached to
  the parse-anchor. It will do so quasi-atomically, that is, two
  different guile threads are guaranteed to get different sentences.
  Its only quasiatomic, because there is no protection against C++
  or python racing to do the same thing.
"
	(get-one-anchored-item (AnchorNode "# New Parsed Sentence"))
)

; -----------------------------------------------------------------------
(define-public (relex-parse plain-txt)
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
			; (system (string-join (list "echo \"Info: send to parser: " sent-txt "\"")))
			(exec-scm-from-port s)
			; (system (string-join (list "echo Info: close socket to parser" )))
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
(define-public (get-parses-of-sents sent-list)
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
(define-public (attach-parses-to-anchor sent-list anchor)
"
  attach-parses-to-anchor -- given sentences, attach the parses to anchor.

  Given a list of sentences i.e. a list of SentenceNodes, go through them,
  locate the ParseNodes, and attach the parse nodes to the anchor.

  return value is undefined (no return value).
"
;  OPENCOG RULE: FYI this could be easily implemented as a pattern match,
;  and probably should be, when processing becomes fully rule-driven.

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

; -----------------------------------------------------------------------
(define-public (parse-all proc path)
"
  Parse of all sentences in each of the files in 'path' using 'proc'. Assuming
  each line of the files represents a sentence.

  'proc': a scheme function for parsing the sentences.

  'path': a string that points to the file or the directory containing the
          sentences to be parsed.
          Node: If it is a directory, all the files including those in its
                sub-directories will be parsed

  Example usage:
      (parse-all nlp-parse \"/home/test/articles\")
"
    (let* ((cmd (string-append "find " path " -type f -exec cat {} \\;"))
           (port (open-input-pipe cmd))
           (line (get-line port))
           (cnt 0))

        (while (not (eof-object? line))
            ; Ignore empty lines
            (if (= (string-length (string-trim line)) 0)
                (set! line (get-line port))
                (begin (catch #t
                    (lambda ()
                        (set! cnt (+ cnt 1))
                        (display (string-append (number->string cnt) ") Parsing: " line "\n"))
                        (proc line))
                    (lambda (key . parameters)
                        (display (string-append "*** Unable to parse: " line))
                        (newline)))
                    (set! line (get-line port)))))
        (display (string-append "Finished parsing " (number->string cnt) " sentences\n"))
        (close-pipe port))
)
