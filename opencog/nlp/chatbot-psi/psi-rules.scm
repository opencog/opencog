(use-modules (opencog) (opencog nlp fuzzy) (opencog openpsi))

(load "states.scm")

;-------------------------------------------------------------------------------
; Define the demands
(define sociality (psi-demand "Sociality"))

;-------------------------------------------------------------------------------
; Define the psi-rules

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "fuzzy-not-started?")
            (DefinedPredicate "is-input-utterance?")
            (SequentialOr
                (Not (DefinedPredicate "input-type-is-imperative?"))
                (DefinedPredicate "don't-know-how-to-do-it?"))
        ))
        (True (ExecutionOutput (GroundedSchema "scm: call-fuzzy") (List)))
        (True)
        (stv .9 .9)
        sociality
    )
    "fuzzy_matcher"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "fuzzy-finished?")
            (DefinedPredicate "is-fuzzy-reply?")
            (SequentialOr
                (Not (DefinedPredicate "input-is-a-question?"))
                (DefinedPredicate "fuzzy-reply-is-declarative?"))
            (DefinedPredicate "no-other-fast-reply?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List fuzzy-reply)))
        (True)
        (stv .9 .9)
        sociality
    )
    "fuzzy_matcher"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "fuzzy-finished?")
            (DefinedPredicate "is-fuzzy-reply?")
            (DefinedPredicate "input-is-a-question?")
            (Not (DefinedPredicate "fuzzy-reply-is-declarative?"))
            (DefinedPredicate "no-good-fast-answer?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List fuzzy-reply)))
        (True)
        (stv .7 .7)
        sociality
    )
    "fuzzy_matcher"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "aiml-not-started?")
            (DefinedPredicate "is-input-utterance?")
            (SequentialOr
                (Not (DefinedPredicate "input-type-is-imperative?"))
                (DefinedPredicate "don't-know-how-to-do-it?"))
        ))
        (True (ExecutionOutput (GroundedSchema "scm: call-aiml") (List)))
        (True)
        (stv 0 .9)
        sociality
    )
    "aiml"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "aiml-finished?")
            (DefinedPredicate "is-aiml-reply?")
            (SequentialOr
                (DefinedPredicate "input-is-about-the-robot?")
                (Not (DefinedPredicate "input-is-a-question?")))
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List aiml-reply)))
        (True)
        (stv 0 .9)
        sociality
    )
    "aiml"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "aiml-finished?")
            (DefinedPredicate "is-aiml-reply?")
            (DefinedPredicate "input-is-a-question?")
            (Not (DefinedPredicate "input-is-about-the-robot?"))
            (SequentialOr
                (DefinedPredicate "is-aiml-reply-good?")
                (DefinedPredicate "no-good-fast-answer?"))
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List aiml-reply)))
        (True)
        (stv 0 .9)
        sociality
    )
    "aiml"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "chatscript-not-started?")
            (DefinedPredicate "is-input-utterance?")
            (Not (DefinedPredicate "asking-how-robot-feels?"))
            (SequentialOr
                (Not (DefinedPredicate "input-type-is-imperative?"))
                (DefinedPredicate "don't-know-how-to-do-it?"))
        ))
        (True (ExecutionOutput (GroundedSchema "scm: call-chatscript") (List)))
        (True)
        (stv .9 .9)
        sociality
    )
    "chatscript"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "chatscript-finished?")
            (DefinedPredicate "is-chatscript-reply?")
            (SequentialOr
                (DefinedPredicate "input-is-about-the-robot?")
                (Not (DefinedPredicate "input-is-a-question?")))
            (DefinedPredicate "no-random-sentence-generator-keywords?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List chatscript-reply)))
        (True)
        (stv .9 .9)
        sociality
    )
    "chatscript"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "chatscript-finished?")
            (DefinedPredicate "is-chatscript-reply?")
            (SequentialOr
                (DefinedPredicate "input-is-about-the-robot?")
                (Not (DefinedPredicate "input-is-a-question?")))
            (DefinedPredicate "has-random-sentence-generator-done-with-the-keywords?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List chatscript-reply)))
        (True)
        (stv .9 .9)
        sociality
    )
    "chatscript"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "chatscript-finished?")
            (DefinedPredicate "is-chatscript-reply?")
            (DefinedPredicate "input-is-a-question?")
            (Not (DefinedPredicate "input-is-about-the-robot?"))
            (DefinedPredicate "no-good-fast-answer?")
            (DefinedPredicate "no-random-sentence-generator-keywords?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List chatscript-reply)))
        (True)
        (stv .9 .9)
        sociality
    )
    "chatscript"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "chatscript-finished?")
            (DefinedPredicate "is-chatscript-reply?")
            (DefinedPredicate "input-is-a-question?")
            (Not (DefinedPredicate "input-is-about-the-robot?"))
            (DefinedPredicate "no-good-fast-answer?")
            (DefinedPredicate "has-random-sentence-generator-done-with-the-keywords?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List chatscript-reply)))
        (True)
        (stv .9 .9)
        sociality
    )
    "chatscript"
)

(psi-set-controlled-rule
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
    "duckduckgo"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "duckduckgo-finished?")
            (DefinedPredicate "is-duckduckgo-answer?")
            (Not (DefinedPredicate "input-is-about-the-robot?"))
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List duckduckgo-answer)))
        (True)
        (stv .9 .9)
        sociality
    )
    "duckduckgo"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "duckduckgo-finished?")
            (DefinedPredicate "is-duckduckgo-answer?")
            (DefinedPredicate "input-is-about-the-robot?")
            (DefinedPredicate "no-other-fast-reply?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List duckduckgo-answer)))
        (True)
        (stv .9 .9)
        sociality
    )
    "duckduckgo"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-wolframalpha-ready?")
            (DefinedPredicate "wolframalpha-not-started?")
            (DefinedPredicate "is-input-utterance?")
            (DefinedPredicate "input-type-is-interrogative?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: ask-wolframalpha") (List)))
        (True)
        (stv .9 .9)
        sociality
    )
    "wolframalpha"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "wolframalpha-finished?")
            (DefinedPredicate "is-wolframalpha-answer?")
            (Not (DefinedPredicate "input-is-about-the-robot?"))
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List wolframalpha-answer)))
        (True)
        (stv .9 .9)
        sociality
    )
    "wolframalpha"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "wolframalpha-finished?")
            (DefinedPredicate "is-wolframalpha-answer?")
            (DefinedPredicate "input-is-about-the-robot?")
            (DefinedPredicate "no-other-fast-reply?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List wolframalpha-answer)))
        (True)
        (stv .9 .9)
        sociality
    )
    "wolframalpha"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-openweathermap-ready?")
            (DefinedPredicate "openweathermap-not-started?")
            (DefinedPredicate "is-input-utterance?")
            (DefinedPredicate "is-weather-related?")
            (DefinedPredicate "input-type-is-interrogative?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: ask-weather") (List)))
        (True)
        (stv .9 .9)
        sociality
    )
    "openweathermap"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "openweathermap-finished?")
            (DefinedPredicate "is-openweathermap-answer?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List openweathermap-answer)))
        (True)
        (stv .9 .9)
        sociality
    )
    "openweathermap"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-random-pkd-sentence-generator-ready?")
            (DefinedPredicate "random-pkd-sentence-generator-not-started?")
            (DefinedPredicate "is-input-utterance?")
            (DefinedPredicate "has-pkd-related-words?")
            (SequentialOr
                (Not (DefinedPredicate "input-type-is-imperative?"))
                (DefinedPredicate "don't-know-how-to-do-it?"))
        ))
        (True (ExecutionOutput (GroundedSchema "scm: call-random-sentence-generator") (List (Node "pkd"))))
        (True)
        (stv .9 .9)
        sociality
    )
    "random_sentence_pkd"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-random-blogs-sentence-generator-ready?")
            (DefinedPredicate "random-blogs-sentence-generator-not-started?")
            (DefinedPredicate "is-input-utterance?")
            (DefinedPredicate "has-blog-related-words?")
            (SequentialOr
                (Not (DefinedPredicate "input-type-is-imperative?"))
                (DefinedPredicate "don't-know-how-to-do-it?"))
        ))
        (True (ExecutionOutput (GroundedSchema "scm: call-random-sentence-generator") (List (Node "blogs"))))
        (True)
        (stv 0 .9)
        sociality
    )
    "random_sentence_blogs"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "is-random-kurzweil-sentence-generator-ready?")
            (DefinedPredicate "random-kurzweil-sentence-generator-not-started?")
            (DefinedPredicate "is-input-utterance?")
            (DefinedPredicate "has-kurzweil-related-words?")
            (SequentialOr
                (Not (DefinedPredicate "input-type-is-imperative?"))
                (DefinedPredicate "don't-know-how-to-do-it?"))
        ))
        (True (ExecutionOutput (GroundedSchema "scm: call-random-sentence-generator") (List (Node "kurzweil"))))
        (True)
        (stv .9 .9)
        sociality
    )
    "random_sentence_kurzweil"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "random-pkd-sentence-generated?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List random-pkd-sentence-generated)))
        (True)
        (stv .9 .9)
        sociality
    )
    "random_sentence_pkd"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "random-blogs-sentence-generated?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List random-blogs-sentence-generated)))
        (True)
        (stv 0 .9)
        sociality
    )
    "random_sentence_blogs"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "random-kurzweil-sentence-generated?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List random-kurzweil-sentence-generated)))
        (True)
        (stv .9 .9)
        sociality
    )
    "random_sentence_kurzweil"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "chatbot-eva-not-started?")
            (DefinedPredicate "is-input-utterance?")
            (DefinedPredicate "input-type-is-imperative?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: call-chatbot-eva") (List)))
        (True)
        (stv .9 .9)
        sociality
    )
    "chatbot_eva"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "chatbot-eva-finished?")
            (DefinedPredicate "going-to-do-the-action?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: say") (List (Word "ok"))))
        (True)
        (stv .9 .9)
        sociality
    )
    "chatbot_eva"
)

; Emotion state inquiry
(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
           (DefinedPredicate "emotion-state-not-started?")
           (DefinedPredicate "is-input-utterance?")
           ;(DefinedPredicate "input-is-a-question?")
           (DefinedPredicate "asking-how-robot-feels?")
        ))
        (True (ExecutionOutput
                (GroundedSchema "scm: call-emotion-state-response") (List)))
        (True)
        (stv .9 .9)
        sociality
    )
    "emotion_state"
)

(psi-set-controlled-rule
    (psi-rule
        (list (SequentialAnd
            (DefinedPredicate "emotion-state-finished?")
            (DefinedPredicate "is-emotion-state-reply?")
            (DefinedPredicate "has-not-replied-anything-yet?")
        ))
        (True (ExecutionOutput (GroundedSchema "scm: reply") (List emotion-state-reply)))
        (True)
        (stv .9 .9)
        sociality
    )
    "emotion-state"
)

(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "input-type-is-interrogative?")
        (DefinedPredicate "is-asking-about-how-many-visible-faces")
        (DefinedPredicate "has-not-replied-anything-yet?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: count-and-reply")
        (List (DefinedSchema "Num visible faces"))))
    (True)
    (stv .9 .9)
    sociality
)

; Return to normal if someone says e.g. "we are done showing it" when it's in
; a demo mode
(psi-rule
    (list (SequentialAnd
        (DefinedPredicate "is-input-utterance?")
        (DefinedPredicate "is-in-any-demo-mode?")
        (DefinedPredicate "is-asked-to-stop-demo?")
    ))
    (True (ExecutionOutput (GroundedSchema "scm: back-to-default-mode") (List)))
    (True)
    (stv .9 .9)
    sociality
)
