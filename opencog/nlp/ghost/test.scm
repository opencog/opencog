; -----
; Tools generally useful for testing / debugging GHOST

(use-modules (opencog nlp chatbot)
             (opencog eva-behavior))

(define-public (test-ghost TXT)
  "Try to find (and execute) the matching rules given an input TXT."
  (define sent (car (nlp-parse TXT)))
  (State (Anchor "GHOST: Currently Processing") sent)
  (map (lambda (r) (cog-evaluate! (gdar r)))
       (cog-outgoing-set (chat-find-rules sent))))

(define-public (ghost-show-lemmas)
  "Show the lemmas stored."
  (display lemma-alist)
  (newline))
