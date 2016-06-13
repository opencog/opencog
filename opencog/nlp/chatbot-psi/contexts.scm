(Define
    (DefinedPredicate "is-input-utterance?")
    (Not (Equal (Set no-input-utterance)
                (Get (State input-utterance (Variable "$x")))))
)

(Define
    (DefinedPredicate "is-declarative?")
    (Satisfaction (And
        (State input-utterance (Reference (Variable "$l") (Variable "$x")))
        (Parse (Variable "$parse") (Variable "$x"))
        (Interpretation (Variable "$interp") (Variable "$parse"))
        (Inheritance (Variable "$interp") (DefinedLinguisticConcept "DeclarativeSpeechAct"))
    ))
)

(Define
    (DefinedPredicate "is-imperative?")
    (Satisfaction (And
        (State input-utterance (Reference (Variable "$l") (Variable "$x")))
        (Parse (Variable "$parse") (Variable "$x"))
        (Interpretation (Variable "$interp") (Variable "$parse"))
        (Inheritance (Variable "$interp") (DefinedLinguisticConcept "ImperativeSpeechAct"))
    ))
)

(Define
    (DefinedPredicate "is-interrogative?")
    (Satisfaction (And
        (State input-utterance (Reference (Variable "$l") (Variable "$x")))
        (Parse (Variable "$parse") (Variable "$x"))
        (Interpretation (Variable "$interp") (Variable "$parse"))
        (Inheritance (Variable "$interp") (DefinedLinguisticConcept "InterrogativeSpeechAct"))
    ))
)

(Define
    (DefinedPredicate "is-truth-query?")
    (Satisfaction (And
        (State input-utterance (Reference (Variable "$l") (Variable "$x")))
        (Parse (Variable "$parse") (Variable "$x"))
        (Interpretation (Variable "$interp") (Variable "$parse"))
        (Inheritance (Variable "$interp") (DefinedLinguisticConcept "TruthQuerySpeechAct"))
    ))
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
