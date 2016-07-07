(use-modules (opencog) (opencog nlp fuzzy) (opencog openpsi))

(load "states.scm")
;-------------------------------------------------------------------------------
; Define the demands

(define sociality (psi-demand "Sociality" .8))
;-------------------------------------------------------------------------------

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "fuzzy-qa-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "is-a-question?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: do-fuzzy-QA") (List)))
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
        (DefinedPredicate "fuzzy-match-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (Not (DefinedPredicate "is-a-question?"))
        (SequentialOr
            (Not (DefinedPredicate "is-imperative?"))
            (DefinedPredicate "don't-know-how-to-do-it")
        )
    ))
    (True (ExecutionOutput (GroundedSchema "scm: do-fuzzy-match") (List)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "fuzzy-match-finished?")
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
        (DefinedPredicate "aiml-search-not-started?")
        (DefinedPredicate "is-input-utterance?")
        (SequentialOr
            (Not (DefinedPredicate "is-imperative?"))
            (DefinedPredicate "don't-know-how-to-do-it")
        )
    ))
    (True (ExecutionOutput (GroundedSchema "scm: do-aiml-search") (List)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "aiml-search-finished?")
        (DefinedPredicate "is-aiml-reply?")
        (DefinedPredicate "is-input-utterance?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: reply") (List aiml-replies)))
    (True)
    (stv .9 .9)
    sociality
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "duckduckgo-search-not-started?")
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
        (DefinedPredicate "duckduckgo-search-finished?")
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
        (DefinedPredicate "wolframalpha-search-not-started?")
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
        (DefinedPredicate "wolframalpha-search-finished?")
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
        (Not (DefinedPredicate "called-chatbot-eva?"))
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "is-imperative?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: call-chatbot-eva") (List)))
    (True)
    (stv .9 .9)
    sociality
)
