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
    (DefinedPredicate "fuzzy-qa-search-started?")
    (search-started? fuzzy-qa-search)
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
