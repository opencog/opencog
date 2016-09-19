(use-modules (opencog) (opencog atom-types))

(load "states.scm")
(load "utils.scm")

;-------------------------------------------------------------------------------
; Useful functions for the contexts

(define (is-utterance-type? speechact)
    (Satisfaction (And
        (State input-utterance-sentence (Variable "$s"))
        (Parse (Variable "$parse") (Variable "$s"))
        (Interpretation (Variable "$interp") (Variable "$parse"))
        (Inheritance (Variable "$interp") speechact)
    ))
)

(define (setup-done? anchor)
    (Not (Equal (Set setup-not-done) (Get
        (TypedVariable (Variable "$s") (Type "ConceptNode"))
            (State anchor (Variable "$s")))))
)

(define (process-not-started? anchor)
    (Equal (Set default-state) (Get
        (TypedVariable (Variable "$s") (Type "ConceptNode"))
            (State anchor (Variable "$s"))))
)

(define (process-finished? anchor)
    (Equal (Set process-finished) (Get
        (TypedVariable (Variable "$s") (Type "ConceptNode"))
            (State anchor (Variable "$s"))))
)

(define (any-result? anchor)
    (Not (Or
        (Equal (Set setup-not-done) (Get
            (TypedVariable (Variable "$f") (Type "ConceptNode"))
                (State anchor (Variable "$f"))))
        (Equal (Set default-state) (Get
            (TypedVariable (Variable "$f") (Type "ConceptNode"))
                (State anchor (Variable "$f"))))
        (Equal (Set no-result) (Get
            (TypedVariable (Variable "$f") (Type "ConceptNode"))
                (State anchor (Variable "$f"))))
    ))
)

(define (no-result? anchor)
    (Equal (Set no-result) (Get
        (TypedVariable (Variable "$r") (Type "ConceptNode"))
            (State anchor (Variable "$r"))))
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

(define (check-fuzzy-reply num-node)
    (if (> (string->number (cog-name (car
            (cog-chase-link 'StateLink 'NumberNode fuzzy-reply-conf))))
                (string->number (cog-name num-node)))
        (stv 1 1)
        (stv 0 1)
    )
)

(define (long-time-elapsed)
    (if (> (- (current-time) (string->number (get-input-time)))
            (string->number (cog-name (gar (cog-execute!
                (Get (TypedVariable (Variable "$t") (Type "TimeNode"))
                    (State max-waiting-time (Variable "$t"))))))))
        (stv 1 1)
        (stv 0 1)
    )
)

(define (check-words . target-words)
    ; target-words is a list of WordNodes
    (if (null? (filter (lambda (w) (list? (member w target-words)))
            (cog-outgoing-set (get-input-word-list))))
        (stv 0 1)
        (stv 1 1)
    )
)

(define rsg-input "")
(define (check-theme target-words)
    (define idx '())
    (define cnt 0)
    (define input-wl (cog-outgoing-set (get-input-word-list)))

    ; target-words is a list of strings
    (filter
        (lambda (w)
            (if (list? (member (cog-name w) target-words))
                (set! idx (append idx (list cnt))))
            (set! cnt (+ cnt 1)))
        input-wl
    )

    (if (null? idx)
        (stv 0 1)
        (let ((rand-idx (list-ref idx (random (length idx)))))
            (if (equal? (length input-wl) 1)
                (set! rsg-input (cog-name (car input-wl)))
                (if (equal? rand-idx 0)
                    (set! rsg-input (string-append
                        (cog-name (list-ref input-wl rand-idx))
                            " " (cog-name (list-ref input-wl (+ rand-idx 1)))))
                    (set! rsg-input (string-append
                        (cog-name (list-ref input-wl (- rand-idx 1)))
                            " " (cog-name (list-ref input-wl rand-idx))))
                )
            )
            (stv 1 1)
        )
    )
)

(define (check-pkd-words)
    (check-theme pkd-relevant-words)
)

(define (check-blog-words)
    (check-theme blog-relevant-words)
)

;-------------------------------------------------------------------------------

(Define
    (DefinedPredicate "is-input-utterance?")
    (Not (Equal (Set no-input-utterance)
        (Get (TypedVariable (Variable "$x") (Type "ConceptNode"))
            (State input-utterance (Variable "$x")))))
)

(Define
    (DefinedPredicate "input-type-is-declarative?")
    (is-utterance-type? (DefinedLinguisticConcept "DeclarativeSpeechAct"))
)

(Define
    (DefinedPredicate "input-type-is-imperative?")
    (is-utterance-type? (DefinedLinguisticConcept "ImperativeSpeechAct"))
)

(Define
    (DefinedPredicate "input-type-is-interrogative?")
    (is-utterance-type? (DefinedLinguisticConcept "InterrogativeSpeechAct"))
)

(Define
    (DefinedPredicate "input-type-is-truth-query?")
    (is-utterance-type? (DefinedLinguisticConcept "TruthQuerySpeechAct"))
)

(Define
    (DefinedPredicate "input-is-a-question?")
    (Or (DefinedPredicate "input-type-is-interrogative?")
        (DefinedPredicate "input-type-is-truth-query?"))
)

(Define
    (DefinedPredicate "input-is-about-the-robot?")
    (Evaluation (GroundedPredicate "scm: check-words")
        (List (Word "you") (Word "your") (Word "yours") (Word "robot")))
)

; Essentially equivalent to "is-input-utterance", as the states
; will be reset after giving a reply by default
(Define
    (DefinedPredicate "has-not-replied-anything-yet?")
    (DefinedPredicate "is-input-utterance?")
)

(Define
    (DefinedPredicate "fuzzy-not-started?")
    (process-not-started? fuzzy)
)

(Define
    (DefinedPredicate "fuzzy-finished?")
    (process-finished? fuzzy)
)

(Define
    (DefinedPredicate "is-fuzzy-reply?")
    (any-result? fuzzy-reply)
)

(Define
    (DefinedPredicate "fuzzy-reply-is-declarative?")
    (Equal (Set (DefinedLinguisticConcept "DeclarativeSpeechAct"))
        (Get (TypedVariable (Variable "$s") (Type "DefinedLinguisticConceptNode"))
            (State fuzzy-reply-type (Variable "$s"))))
)

(Define
    (DefinedPredicate "is-fuzzy-reply-good?")
    (Evaluation (GroundedPredicate "scm: check-fuzzy-reply") (List (Number .5)))
)

(Define
    (DefinedPredicate "no-other-fast-reply?")
    ; TODO: May want to check more than time elapsed
    (Evaluation (GroundedPredicate "scm: long-time-elapsed") (List))
)

(Define
    (DefinedPredicate "aiml-not-started?")
    (process-not-started? aiml)
)

(Define
    (DefinedPredicate "aiml-finished?")
    (process-finished? aiml)
)

; Number being passed is the confidence threshold
(Define
    (DefinedPredicate "is-aiml-reply-good?")
    (Evaluation (GroundedPredicate "scm: check-aiml-reply") (List (Number .5)))
)

(Define
    (DefinedPredicate "no-result-from-other-sources?")
    (And (process-finished? duckduckgo)
         (process-finished? wolframalpha)
         (process-finished? pln-qa)
         (no-result? duckduckgo-answer)
         (no-result? wolframalpha-answer)
         (no-result? pln-answers))
)

(Define
    (DefinedPredicate "no-good-fast-answer?")
    (Or (DefinedPredicate "no-result-from-other-sources?")
        (Evaluation (GroundedPredicate "scm: long-time-elapsed") (List)))
)

(Define
    (DefinedPredicate "is-aiml-reply?")
    (any-result? aiml-reply)
)

(Define
    (DefinedPredicate "duckduckgo-not-started?")
    (process-not-started? duckduckgo)
)

(Define
    (DefinedPredicate "duckduckgo-finished?")
    (process-finished? duckduckgo)
)

(Define
    (DefinedPredicate "is-duckduckgo-answer?")
    (any-result? duckduckgo-answer)
)

(Define
    (DefinedPredicate "is-wolframalpha-ready?")
    (setup-done? wolframalpha)
)

(Define
    (DefinedPredicate "wolframalpha-not-started?")
    (process-not-started? wolframalpha)
)

(Define
    (DefinedPredicate "wolframalpha-finished?")
    (process-finished? wolframalpha)
)

(Define
    (DefinedPredicate "is-wolframalpha-answer?")
    (any-result? wolframalpha-answer)
)

(Define
    (DefinedPredicate "is-random-pkd-sentence-generator-ready?")
    (setup-done? random-pkd-sentence-generator)
)

(Define
    (DefinedPredicate "is-random-blogs-sentence-generator-ready?")
    (setup-done? random-blogs-sentence-generator)
)

(Define
    (DefinedPredicate "random-pkd-sentence-generator-not-started?")
    (process-not-started? random-pkd-sentence-generator)
)

(Define
    (DefinedPredicate "random-blogs-sentence-generator-not-started?")
    (process-not-started? random-blogs-sentence-generator)
)

(Define
    (DefinedPredicate "random-pkd-sentence-generated?")
    (any-result? random-pkd-sentence-generated)
)

(Define
    (DefinedPredicate "random-blogs-sentence-generated?")
    (any-result? random-blogs-sentence-generated)
)

(Define
    (DefinedPredicate "has-pkd-related-words?")
    (Evaluation (GroundedPredicate "scm: check-pkd-words") (List))
)

(Define
    (DefinedPredicate "has-blog-related-words?")
    (Evaluation (GroundedPredicate "scm: check-blog-words") (List))
)

(Define
    (DefinedPredicate "chatbot-eva-not-started?")
    (process-not-started? chatbot-eva)
)

(Define
    (DefinedPredicate "chatbot-eva-finished?")
    (process-finished? chatbot-eva)
)

(Define
    (DefinedPredicate "going-to-do-the-action?")
    (Not (Equal (Set no-result) (Get
        (TypedVariable (Variable "$s") (Type "ConceptNode"))
            (State chatbot-eva-action (Variable "$s")))))
)

(Define
    (DefinedPredicate "don't-know-how-to-do-it?")
    (Or (Evaluation (GroundedPredicate "scm: long-time-elapsed") (List))
        (Equal (Set no-result) (Get
            (TypedVariable (Variable "$s") (Type "ConceptNode"))
                (State chatbot-eva-action (Variable "$s")))))
)
