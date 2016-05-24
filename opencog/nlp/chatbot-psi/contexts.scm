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

(define (did-someone-say-this? . words)
    ; TODO: Similar to "do-fuzzy-match"
    (stv 1 1)
)

(Define
    (DefinedPredicate "fuzzy-qa-search-not-started?")
    (Not (Equal (Set search-started)
                (Get (State fuzzy-qa-search (Variable "$s")))))
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
    (DefinedPredicate "no-canned-reply?")
    (Equal (Set no-canned-rules)
           (Get (State canned-rules (Variable "$r"))))
)
