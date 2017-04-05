(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define pos (pos "cat" "noun"))

; Just want to see if the PartOfSpeechLink and LemmaLink are there...
(define test-result
    (equal? 2 (length (filter (lambda (x)
        (or (and (eq? (cog-type x) 'PartOfSpeechLink)
                 (equal? (gdr x) (DefinedLinguisticConceptNode "noun")))
            (and (eq? (cog-type x) 'LemmaLink)
                 (equal? (gdr x) (Word "cat")))))
        (cdr pos)))))
