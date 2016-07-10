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
        (DefinedPredicate "fuzzy-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (SequentialOr
            (Not (DefinedPredicate "input-type-is-imperative?"))
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
        (SequentialOr
            (Not (DefinedPredicate "input-is-a-question?"))
            (DefinedPredicate "fuzzy-reply-is-declarative?")
        )
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List fuzzy-reply)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "fuzzy-finished?")
        (DefinedPredicate "is-fuzzy-reply?")
        (DefinedPredicate "input-is-a-question?")
        (Not (DefinedPredicate "fuzzy-reply-is-declarative?"))
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List fuzzy-reply)))
    (True)
    (stv .9 .5)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "aiml-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (SequentialOr
            (Not (DefinedPredicate "input-type-is-imperative?"))
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
            (Not (DefinedPredicate "input-is-a-question?"))
            (DefinedPredicate "has-not-replied-anything-yet?")
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
            (DefinedPredicate "input-is-a-question?")
            (Or (DefinedPredicate "is-aiml-reply-good?")
                (DefinedPredicate "no-good-fast-answer?"))
            (DefinedPredicate "has-not-replied-anything-yet?")
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
        (DefinedPredicate "input-is-a-question?")
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
        (DefinedPredicate "has-not-replied-anything-yet?")
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
        (DefinedPredicate "input-type-is-interrogative?")
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
        (DefinedPredicate "has-not-replied-anything-yet?")
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
        (DefinedPredicate "has-not-replied-anything-yet?")
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
        (DefinedPredicate "input-type-is-imperative?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: call-chatbot-eva") (List)))
    (True)
    (stv .9 .9)
    sociality
)
