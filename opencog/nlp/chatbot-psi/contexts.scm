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
