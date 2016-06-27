(use-modules (ice-9 threads))
(use-modules (opencog)
             (opencog nlp)
             (opencog nlp relex2logic)
             (opencog nlp fuzzy)
             (opencog nlp sureal)
             (opencog nlp chatbot-eva)
             (opencog nlp aiml)
             (opencog exec)
             (opencog openpsi))

(load-r2l-rulebase)

;-------------------------------------------------------------------------------
; Schema function for chatting

(define (chat utterance)
    (reset-all-states)

    (let ((sent-node (car (nlp-parse utterance))))
        (State input-utterance
            (Reference
                sent-node
                (Node utterance)
                (get-word-list sent-node)
            )
        )
    )

    *unspecified*
)

;-------------------------------------------------------------------------------
; Keep track of the states

(define (chat-prefix node_name) (string-append "Chatbot: " node_name))

(define input-utterance (Anchor (chat-prefix "InputUtterance")))
(define no-input-utterance (Concept (chat-prefix "NoInputUtterance")))
(State input-utterance no-input-utterance)

(define input-parse (Anchor (chat-prefix "InputParse")))
(define parse-succeeded (Concept (chat-prefix "ParseSucceeded")))
(define parse-failed (Concept (chat-prefix "ParseFailed")))
(State input-parse no-input-utterance)

(define default-state (Concept (chat-prefix "DefaultState")))
(define search-started (Concept (chat-prefix "SearchStarted")))
(define no-result (Concept (chat-prefix "NoResult")))

(define aiml-replies (Anchor (chat-prefix "AIMLReplies")))
(define aiml-search (Anchor (chat-prefix "AIMLSearch")))
(State aiml-replies default-state)
(State aiml-search default-state)

(define fuzzy-replies (Anchor (chat-prefix "FuzzyReplies")))
(define fuzzy-match (Anchor (chat-prefix "FuzzyMatch")))
(State fuzzy-replies default-state)
(State fuzzy-match default-state)

(define fuzzy-answers (Anchor (chat-prefix "FuzzyAnswers")))
(define fuzzy-qa (Anchor (chat-prefix "FuzzyQA")))
(State fuzzy-answers default-state)
(State fuzzy-qa default-state)

(define duckduckgo-answers (Anchor (chat-prefix "DuckDuckGoAnswers")))
(define duckduckgo-search (Anchor (chat-prefix "DuckDuckGoSearch")))
(State duckduckgo-answers default-state)
(State duckduckgo-search default-state)

(define chatbot-eva (Anchor (chat-prefix "ChatbotEva")))
(define sent-to-chatbot-eva (Concept (chat-prefix "SentToChatbotEva")))
(define no-action-taken (Concept (chat-prefix "NoActionTaken")))
(State chatbot-eva default-state)

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
