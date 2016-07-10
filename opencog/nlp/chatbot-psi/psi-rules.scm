(use-modules (opencog) (opencog nlp fuzzy) (opencog openpsi))

(load "states.scm")

;-------------------------------------------------------------------------------
; Define the demands

(define sociality (psi-demand "Sociality" .8))

;-------------------------------------------------------------------------------
; Define the tags

(define aiml-reply-rule (Concept (chat-prefix "AIMLReplyRule")))

;-------------------------------------------------------------------------------
; Define the psi-rules

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "fuzzy-qa-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "is-a-question?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: call-fuzzy-QA") (List)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "fuzzy-qa-finished?")
        (DefinedPredicate "is-fuzzy-answer?")
        (DefinedPredicate "is-input-utterance?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List fuzzy-answers)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "fuzzy-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (Not (DefinedPredicate "is-a-question?"))
        (SequentialOr
            (Not (DefinedPredicate "is-imperative?"))
            (DefinedPredicate "don't-know-how-to-do-it")
        )
    ))
    (True (ExecutionOutput (GroundedSchema "scm: call-fuzzy") (List)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "fuzzy-finished?")
        (DefinedPredicate "is-fuzzy-reply?")
        (DefinedPredicate "is-input-utterance?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List fuzzy-replies)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "aiml-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (SequentialOr
            (Not (DefinedPredicate "is-imperative?"))
            (DefinedPredicate "don't-know-how-to-do-it")
        )
    ))
    (True (ExecutionOutput (GroundedSchema "scm: call-aiml") (List)))
    (True)
    (stv .9 .9)
    sociality
)

(Member
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "aiml-finished?")
            (DefinedPredicate "is-aiml-reply?")
            (Not (DefinedPredicate "is-a-question?"))
            (DefinedPredicate "is-input-utterance?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List aiml-replies)))
        (True)
        (stv .9 .9)
        sociality
    )
    aiml-reply-rule
)

(Member
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "aiml-finished?")
            (DefinedPredicate "is-aiml-reply?")
            (DefinedPredicate "is-a-question?")
            (Or (DefinedPredicate "is-aiml-reply-good?")
                (DefinedPredicate "no-good-fast-answer?"))
            (DefinedPredicate "is-input-utterance?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List aiml-replies)))
        (True)
        (stv .9 .9)
        sociality
    )
    aiml-reply-rule
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "duckduckgo-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "is-a-question?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: ask-duckduckgo") (List)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "duckduckgo-finished?")
        (DefinedPredicate "is-duckduckgo-answer?")
        (DefinedPredicate "is-input-utterance?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List duckduckgo-answers)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "wolframalpha-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "is-interrogative?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: ask-wolframalpha") (List)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "wolframalpha-finished?")
        (DefinedPredicate "is-wolframalpha-answer?")
        (DefinedPredicate "is-input-utterance?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List wolframalpha-answers)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "random-sentence-generator-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "has-pkd-related-words?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: call-random-sentence-generator") (List (Node "pkd"))))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "random-sentence-generator-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "has-blog-related-words?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: call-random-sentence-generator") (List (Node "blogs"))))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "random-sentence-generated?")
        (DefinedPredicate "is-input-utterance?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List random-sentence-generated)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (Not (DefinedPredicate "called-chatbot-eva?"))
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "is-imperative?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: call-chatbot-eva") (List)))
    (True)
    (stv .9 .9)
    sociality
)
