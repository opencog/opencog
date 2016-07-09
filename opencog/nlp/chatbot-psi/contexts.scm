(use-modules (opencog) (opencog atom-types))

(load "states.scm")
(load "utils.scm")

;-------------------------------------------------------------------------------
; Useful functions for the contexts

(define (is-utterance-type? speechact)
    (Satisfaction (And
        (State input-utterance (Reference (Variable "$s") (Variable "$n") (Variable "$l")))
        (Parse (Variable "$parse") (Variable "$s"))
        (Interpretation (Variable "$interp") (Variable "$parse"))
        (Inheritance (Variable "$interp") speechact)
    ))
)

(define (process-not-started? anchor)
    (Equal (Set default-state) (Get (State anchor (Variable "$s"))))
)

(define (process-finished? anchor)
    (Equal (Set process-finished) (Get (State anchor (Variable "$s"))))
)

(define (any-result? anchor)
    (Not (Or
        (Equal (Set default-state) (Get (State anchor (Variable "$f"))))
        (Equal (Set no-result) (Get (State anchor (Variable "$f"))))
    ))
)

(define (no-result? anchor)
    (Equal (Set no-result) (Get (State anchor (Variable "$r"))))
)

(define (check-aiml-reply num-node)
    (let* ((tv (cog-tv (aiml-get-selected-rule)))
           (conf (cog-tv-confidence tv))
           (threshold (string->number (cog-name num-node))))
        (if (> conf threshold)
            (stv 1 1)
            (stv 0 1)
        )
    )
)

(define (long-time-elapsed num-node)
    (if (> (- (current-time) (string->number (get-input-time)))
            (string->number (cog-name num-node)))
        (stv 1 1)
        (stv 0 1)
    )
)

;-------------------------------------------------------------------------------

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
    (Or (DefinedPredicate "is-interrogative?")
        (DefinedPredicate "is-truth-query?"))
)

(Define
    (DefinedPredicate "fuzzy-qa-not-started?")
    (process-not-started? fuzzy-qa)
)

(Define
    (DefinedPredicate "fuzzy-qa-finished?")
    (process-finished? fuzzy-qa)
)

(Define
    (DefinedPredicate "is-fuzzy-answer?")
    (any-result? fuzzy-answers)
)

(Define
    (DefinedPredicate "fuzzy-match-not-started?")
    (process-not-started? fuzzy-match)
)

(Define
    (DefinedPredicate "fuzzy-match-finished?")
    (process-finished? fuzzy-match)
)

(Define
    (DefinedPredicate "is-fuzzy-reply?")
    (any-result? fuzzy-replies)
)

(Define
    (DefinedPredicate "aiml-search-not-started?")
    (process-not-started? aiml-search)
)

(Define
    (DefinedPredicate "aiml-search-finished?")
    (process-finished? aiml-search)
)

; Number being passed is the confidence threshold
(Define
    (DefinedPredicate "is-aiml-reply-good?")
    (Evaluation (GroundedPredicate "scm: check-aiml-reply") (List (Number .5)))
)

(Define
    (DefinedPredicate "no-result-from-other-sources?")
    (And (process-finished? duckduckgo-search)
         (process-finished? wolframalpha-search)
         (no-result? duckduckgo-answers)
         (no-result? wolframalpha-answers))
)

; Number being passed is time (in second) threshold
(Define
    (DefinedPredicate "no-good-fast-answer?")
    (Or (DefinedPredicate "no-result-from-other-sources?")
        (Evaluation (GroundedPredicate "scm: long-time-elapsed") (List (Number 3))))
)

(Define
    (DefinedPredicate "is-aiml-reply?")
    (any-result? aiml-replies)
)

(Define
    (DefinedPredicate "duckduckgo-search-not-started?")
    (process-not-started? duckduckgo-search)
)

(Define
    (DefinedPredicate "duckduckgo-search-finished?")
    (process-finished? duckduckgo-search)
)

(Define
    (DefinedPredicate "is-duckduckgo-answer?")
    (any-result? duckduckgo-answers)
)

(Define
    (DefinedPredicate "wolframalpha-search-not-started?")
    (process-not-started? wolframalpha-search)
)

(Define
    (DefinedPredicate "wolframalpha-search-finished?")
    (process-finished? wolframalpha-search)
)

(Define
    (DefinedPredicate "is-wolframalpha-answer?")
    (any-result? wolframalpha-answers)
)

(Define
    (DefinedPredicate "random-sentence-generator-not-started?")
    (process-not-started? random-sentence-generator)
)

(Define
    (DefinedPredicate "random-sentence-generated?")
    (any-result? random-sentence-generated)
)

(Define
    (DefinedPredicate "has-pkd-related-words?")
    ; TODO
    (True)
)

(Define
    (DefinedPredicate "has-blog-related-words?")
    ; TODO
    (True)
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
