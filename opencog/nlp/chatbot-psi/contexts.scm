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
    (DefinedPredicate "fuzzy-qa-search-started?")
    (Equal (Set search-started)
           (Get (State fuzzy-qa-search (Variable "$s"))))
)

(Define
    (DefinedPredicate "is-a-question?")
    (Satisfaction (Or
        (DefinedPredicate "is-interrogative?")
        (DefinedPredicate "is-truth-query?")
    ))
)

(Define
    (DefinedPredicate "is-fuzzy-answer?")
    (Not (Equal (Set no-fuzzy-answers)
                (Get (State fuzzy-answers (Variable "$f")))))
)

(Define
    (DefinedPredicate "fuzzy-search-started?")
    (Equal (Set search-started)
           (Get (State fuzzy-search (Variable "$s"))))
)

(Define
    (DefinedPredicate "is-fuzzy-reply?")
    (Not (Equal (Set no-fuzzy-reply)
                (Get (State fuzzy-replies (Variable "$r")))))
)

(Define
    (DefinedPredicate "aiml-search-started?")
    (Equal (Set search-started)
           (Get (State aiml-search (Variable "$s"))))
)

(Define
    (DefinedPredicate "is-aiml-reply?")
    (Not (Equal (Set no-aiml-reply)
                (Get (State aiml-replies (Variable "$r")))))
)
