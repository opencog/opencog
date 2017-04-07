(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog openpsi))

(define rule (chat-rule '((proper-names "James" "Gilbert")
                          (lemma "like")
                          (concept "watch")
                          (or-choices "movie" "TV")
                          (word "and")
                          (main-subj "Richard")
                          (main-verb "eat")
                          (pos "delicious" "adj")
                          (main-obj "fish"))
                        '(say "I'm surprised!")))

; This is the conditions under the AndLink of the SatisfactionLink
(define conditions (cog-outgoing-set (gdr (car (psi-get-context rule)))))

; Each of the terms should have been tested already by now, here
; just check some trivial things... Expand it if needed
(define test-chat-rule-result
    (and (equal? (cog-type rule) 'ImplicationLink)
         (psi-rule? rule)
         ; Check if the conditions below are generated in the rule
         ; This is needed for DualLink to find the rule
         (list? (member (True (List (Word "James")
                                    (Word "Gilbert")
                                    (Word "like")
                                    (Glob "watch")
                                    (Glob "$choices")
                                    (Word "and")
                                    (Word "Richard")
                                    (Word "eat")
                                    (Word "delicious")
                                    (Word "fish")))
                        conditions))
         ; This is needed one of the shared conditions
         (list? (member (State (Anchor "Currently Processing") (Variable "$S"))
                        conditions))
         ; This is another shared conditions
         (list? (member (Parse (Variable "$P") (Variable "$S"))
                        conditions))))
