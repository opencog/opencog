(Define
    (DefinedPredicate "is-input-utterance?")
    (Not (Equal (Set no-input-utterance)
                (Get (State input-utterance (Variable "$x")))))
)

(Define
    (DefinedPredicate "is-declarative?")
    (is-utterance-type? (DefinedLinguisticConcept "DeclarativeSpeechAct"))
)

(Define
    (DefinedPredicate "is-imperative?")
    (is-utterance-type? (DefinedLinguisticConcept "ImperativeSpeechAct"))
)

(Define
    (DefinedPredicate "is-interrogative?")
    (is-utterance-type? (DefinedLinguisticConcept "InterrogativeSpeechAct"))
)

(Define
    (DefinedPredicate "is-truth-query?")
    (is-utterance-type? (DefinedLinguisticConcept "TruthQuerySpeechAct"))
)

(Define
    (DefinedPredicate "is-a-question?")
    (Satisfaction (Or
        (DefinedPredicate "is-interrogative?")
        (DefinedPredicate "is-truth-query?")
    ))
)

(Define
    (DefinedPredicate "fuzzy-qa-started?")
    (search-started? fuzzy-qa)
)

(Define
    (DefinedPredicate "is-fuzzy-answer?")
    (any-result? fuzzy-answers)
)

(Define
    (DefinedPredicate "fuzzy-match-started?")
    (search-started? fuzzy-match)
)

(Define
    (DefinedPredicate "is-fuzzy-reply?")
    (any-result? fuzzy-replies)
)

(Define
    (DefinedPredicate "aiml-search-started?")
    (search-started? aiml-search)
)

(Define
    (DefinedPredicate "is-aiml-reply?")
    (any-result? aiml-replies)
)

(Define
    (DefinedPredicate "duckduckgo-search-started?")
    (search-started? duckduckgo-search)
)

(Define
    (DefinedPredicate "is-duckduckgo-answer?")
    (any-result? duckduckgo-answers)
)

(Define
    (DefinedPredicate "called-chatbot-eva?")
    (Not (Or
        (Equal (Set default-state) (Get (State chatbot-eva (Variable "$s"))))
        (Equal (Set no-action-taken) (Get (State chatbot-eva (Variable "$s"))))
    ))
)

(Define
    (DefinedPredicate "don't-know-how-to-do-it")
    (Equal (Set no-action-taken) (Get (State chatbot-eva (Variable "$s"))))
)
