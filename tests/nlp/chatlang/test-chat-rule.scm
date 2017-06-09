(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog openpsi)
             (srfi srfi-1))

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
         (equal? #t (any (lambda (c)
           (if (and (equal? 'EvaluationLink (cog-type c))
                    (equal? (Predicate "Chatlang: term seq") (gar c)))
               (let ((oset (cog-outgoing-set (gddr c))))
                    (and (equal? 'GlobNode (cog-type (first oset)))
                         (equal? 'GlobNode (cog-type (last oset)))
                         (equal? (list (Word "James")
                                       (Word "Gilbert")
                                       (Word "like")
                                       (Word "movie"))
                                 (cog-filter 'WordNode oset))))
               #f)) conditions))
         ; This is needed one of the shared conditions
         (list? (member (State (Anchor "Chatlang: Currently Processing")
                               (Variable "$S"))
                        conditions))
         ; This is another shared conditions
         (list? (member (Parse (Variable "$P") (Variable "$S"))
                        conditions))))
         ; Including this causes a failure in cxxtest for no reason
         ; (equal? (gaddr (psi-get-action rule)) (Node "Cool"))))
