(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define or-choices (or-choices "work" "play"))

; Just want to see if the ChoiceLink is there...
(define test-or-choices-result
    (not (null? (filter (lambda (x)
        (and (eq? (cog-type x) 'ChoiceLink)
             (and (eq? (cog-type (gar x)) 'LemmaLink)
                  (equal? (gdr (gar x)) (WordNode "work")))
             (and (eq? (cog-type (gdr x)) 'LemmaLink)
                  (equal? (gdr (gdr x)) (WordNode "play")))))
        (cdr or-choices)))))
