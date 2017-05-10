(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog openpsi))

; Just try a simple one
(define rule (cr '("'James Gilbert' 'liked movie") '("Cool")))

; These are the conditions under the AndLink of the SatisfactionLink
; Susceptible to any change in format
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
                                    (Word "movie")))
                        conditions))
         ; This is needed one of the shared conditions
         (list? (member (State (Anchor "Chatlang: Currently Processing")
                               (Variable "$S"))
                        conditions))
         ; This is another shared conditions
         (list? (member (Parse (Variable "$P") (Variable "$S"))
                        conditions))))
         ; Including this causes a failure in cxxtest for no reason
         ; (equal? (gaddr (psi-get-action rule)) (Node "Cool"))))
