; -----
; Tools generally useful for testing / debugging GHOST

(use-modules (opencog nlp chatbot))

(define-public (ghost-debug-mode)
  (cog-logger-set-level! ghost-logger "debug")
  (cog-logger-set-stdout! ghost-logger #t))

(define-public (ghost-parse TXT)
  "Parse the TXT, convert them into atomese."
  (test-parse TXT))

(define-public (ghost-parse-file FILE)
  "Parse everything in FILE, convert them into atomese."
  (test-parse-file FILE))

(define-public (test-ghost TXT)
  "Try to find (and execute) the matching rules given an input TXT."
  (define sent (car (nlp-parse TXT)))
  (State (Anchor "GHOST: Currently Processing") sent)
  (map (lambda (r) (cog-evaluate! (gdar r)))
       (cog-outgoing-set (chat-find-rules sent)))
  *unspecified*)

(define-public (ghost-show-lemmas)
  "Show the lemmas stored."
  (display lemma-alist)
  (newline))

(define-public (ghost-show-vars)
  "Show the groundings of variables stored."
  (display "=== Variables (words)\n") (display var-grd-words) (newline)
  (display "=== Variables (lemmas)\n") (display var-grd-lemmas) (newline)
  (display "=== User Variables\n") (display uvars) (newline))

(define-public (ghost-get-curr-sent)
  "Get the SentenceNode that is being processed currently."
  (define sent (cog-chase-link 'StateLink 'SentenceNode ghost-anchor))
  (if sent (car sent) '()))

(define*-public (ghost-show-relation #:optional (SENT (ghost-get-curr-sent)))
  "Get a subset of the RelEx outputs of a sentence that GHOST cares.
   SENT is a SentenceNode, if not given, it will be the current input."
  (define parses (sentence-get-parses SENT))
  (define relex-outputs (append-map parse-get-relex-outputs parses))
  (filter
    (lambda (r)
      (define type (cog-type r))
      (or (equal? 'ParseLink type)
          (equal? 'WordInstanceLink type)
          (equal? 'ReferenceLink type)
          (equal? 'LemmaLink type)))
    relex-outputs))
