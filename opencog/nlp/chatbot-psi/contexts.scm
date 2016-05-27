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
    (let* ((rule-sent-node (car (cog-chase-link 'ReferenceLink 'SentenceNode (List words))))
           (rule-r2l (get-r2l-set-of-sent rule-sent-node))
           (input-sent-node (get-input-sent-node))
           (input-r2l (get-r2l-set-of-sent input-sent-node))
           (score (string->number (cog-name (nlp-fuzzy-compare input-r2l rule-r2l)))))
        (stv 1 score)
    )
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
    (DefinedPredicate "no-canned-reply?")
    (Equal (Set no-canned-rules)
           (Get (State canned-rules (Variable "$r"))))
)
