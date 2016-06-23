(use-modules (ice-9 threads))
(use-modules (opencog)
             (opencog nlp)
             (opencog nlp relex2logic)
             (opencog nlp fuzzy)
             (opencog nlp sureal)
             (opencog nlp chatbot)
             (opencog nlp aiml)
             (opencog exec)
             (opencog openpsi))

(load-r2l-rulebase)

;-------------------------------------------------------------------------------
; Schema function for chatting

(define (chat utterance)
    (cancel-all-threads)
    (reset-all-states)

    (catch #t
        (lambda ()
            (let ((sent-node (car (nlp-parse utterance))))
                (State input-parse parse-succeeded)
                (State input-utterance
                    (Reference
                        sent-node
                        (Node utterance)
                        (get-word-list sent-node)
                    )
                )
            ))
        (lambda (key . parameters)
            (State input-parse parse-failed)
        )
    )

    *unspecified*
)

;-------------------------------------------------------------------------------
; Keep track of the states

(define all-threads '())
(define chat-prefix "Chatbot: ")

(define input-utterance (Anchor (string-append chat-prefix "InputUtterance")))
(define no-input-utterance (Concept (string-append chat-prefix "NoInputUtterance")))
(State input-utterance no-input-utterance)

(define input-parse (Anchor (string-append chat-prefix "InputParse")))
(define parse-succeeded (Concept (string-append chat-prefix "ParseSucceeded")))
(define parse-failed (Concept (string-append chat-prefix "ParseFailed")))
(State input-parse no-input-utterance)

(define default-state (Concept (string-append chat-prefix "DefaultState")))
(define search-started (Concept (string-append chat-prefix "SearchStarted")))
(define no-result (Concept (string-append chat-prefix "NoResult")))

(define aiml-replies (Anchor (string-append chat-prefix "AIMLReplies")))
(define aiml-search (Anchor (string-append chat-prefix "AIMLSearch")))
(State aiml-replies default-state)
(State aiml-search default-state)

(define fuzzy-replies (Anchor (string-append chat-prefix "FuzzyReplies")))
(define fuzzy-match (Anchor (string-append chat-prefix "FuzzyMatch")))
(State fuzzy-replies default-state)
(State fuzzy-match default-state)

(define fuzzy-answers (Anchor (string-append chat-prefix "FuzzyAnswers")))
(define fuzzy-qa-search (Anchor (string-append chat-prefix "FuzzyQASearch")))
(State fuzzy-answers default-state)
(State fuzzy-qa-search default-state)

(define duckduckgo-answers (Anchor (string-append chat-prefix "DuckDuckGoAnswers")))
(define duckduckgo-search (Anchor (string-append chat-prefix "DuckDuckGoSearch")))
(State duckduckgo-answers default-state)
(State duckduckgo-search default-state)

;-------------------------------------------------------------------------------
; Define the demands

(define sociality (psi-demand "Sociality" .8))

;-------------------------------------------------------------------------------

; Load the utilities
(load "utils.scm")

; Load the available contexts
(load "contexts.scm")

; Load the available actions
(load "actions.scm")
(load "duckduckgo.scm")

; Load the psi-rules
(load "psi-rules.scm")

; Run OpenPsi if it's not already running
(if (not (psi-running?))
    (psi-run)
)
