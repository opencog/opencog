;-------------------------------------------------------------------------------

(define chat-rule (Concept (string-append (psi-prefix-str) "chat-rule")))
(define canned-rule (Concept (string-append (psi-prefix-str) "canned-rule")))

;-------------------------------------------------------------------------------

(Member
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "fuzzy-qa-search-not-started?")
            (DefinedPredicate "is-input-utterance?")
            (DefinedPredicate "no-canned-reply?")
            (DefinedPredicate "is-interrogative?")
        ))
        (ExecutionOutput (GroundedSchema "scm: do-fuzzy-QA") (List))
        (True)
        (stv 1 1)
        sociality
    )
    chat-rule
)

(Member
    (psi-rule
        (list (DefinedPredicate "is-fuzzy-answer?"))
        (ExecutionOutput (GroundedSchema "scm: answer") (List fuzzy-answers))
        (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    chat-rule
)

;-------------------------------------------------------------------------------
; For testing purpose only
; All canned rules will be generated and imported separately

(define (get-words sent)
    (let ((words (car (sent-get-words-in-order sent))))
        (append-map
            (lambda (w)
                (if (not (string-prefix? "LEFT-WALL" (cog-name w)))
                    (cog-chase-link 'ReferenceLink 'WordNode w) '()
                )
            )
            words
        )
    )
)

;----------
; Input: Are you conscious?

(define in_utt (get-words (car (nlp-parse "Are you conscious?"))))
(define out_utt_1 (get-words (car (nlp-parse "I'm as conscious as you are, meat machine!"))))
(define out_utt_2 (get-words (car (nlp-parse "Yes I am."))))
(define out_utt_3 (get-words (car (nlp-parse "What do you think?"))))
(define out_utt_4 (get-words (car (nlp-parse "Of course."))))
(define out_utt_5 (get-words (car (nlp-parse "Let me check."))))
(Member
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-input-utterance?")
            (Evaluation (GroundedPredicate "scm: did-someone-say-this?") (List in_utt))
        ))
        (ExecutionOutput (GroundedSchema "scm: say") (List out_utt_1))
        (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    canned-rule
)
(Member
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-input-utterance?")
            (Evaluation (GroundedPredicate "scm: did-someone-say-this?") (List in_utt))
        ))
        (ExecutionOutput (GroundedSchema "scm: say") (List out_utt_2))
        (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    canned-rule
)
(Member
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-input-utterance?")
            (Evaluation (GroundedPredicate "scm: did-someone-say-this?") (List in_utt))
        ))
        (ExecutionOutput (GroundedSchema "scm: say") (List out_utt_3))
        (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    canned-rule
)
(Member
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-input-utterance?")
            (Evaluation (GroundedPredicate "scm: did-someone-say-this?") (List in_utt))
        ))
        (ExecutionOutput (GroundedSchema "scm: say") (List out_utt_4))
        (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    canned-rule
)
(Member
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-input-utterance?")
            (Evaluation (GroundedPredicate "scm: did-someone-say-this?") (List in_utt))
        ))
        (ExecutionOutput (GroundedSchema "scm: say") (List out_utt_5))
        (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    canned-rule
)

;----------
; Input: What is your name?

(define in_utt (get-words (car (nlp-parse "What is your name?"))))
(define out_utt_1 (get-words (car (nlp-parse "My name is Red."))))
(define out_utt_2 (get-words (car (nlp-parse "I am Red."))))
(define out_utt_3 (get-words (car (nlp-parse "Red."))))
(Member
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-input-utterance?")
            (Evaluation (GroundedPredicate "scm: did-someone-say-this?") (List in_utt))
        ))
        (ExecutionOutput (GroundedSchema "scm: say") (List out_utt_1))
        (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    canned-rule
)
(Member
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-input-utterance?")
            (Evaluation (GroundedPredicate "scm: did-someone-say-this?") (List in_utt))
        ))
        (ExecutionOutput (GroundedSchema "scm: say") (List out_utt_2))
        (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    canned-rule
)
(Member
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-input-utterance?")
            (Evaluation (GroundedPredicate "scm: did-someone-say-this?") (List in_utt))
        ))
        (ExecutionOutput (GroundedSchema "scm: say") (List out_utt_3))
        (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    canned-rule
)
