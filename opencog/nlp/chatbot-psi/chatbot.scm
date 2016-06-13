(use-modules (ice-9 threads))
(use-modules (opencog)
             (opencog nlp)
             (opencog nlp relex2logic)
             (opencog nlp fuzzy)
             (opencog nlp chatbot)
             (opencog nlp aiml)
             (opencog exec)
             (opencog openpsi))

(load-r2l-rulebase)

;-------------------------------------------------------------------------------
; Schema function for chatting

(define (chat utterance)
    (reset-all-states)

    (let* ((sent-node (car (nlp-parse utterance)))
           (list-of-words (get-word-list sent-node)))

        (State input-utterance
            (Reference
                list-of-words
                sent-node
            )
        )
    )
    (newline)
)

;-------------------------------------------------------------------------------
; Keep track of the states

(define input-utterance (Anchor "InputUtterance"))
(define no-input-utterance (Concept "NoInputUtterance"))
(State input-utterance no-input-utterance)

(define default-state (Concept "DefaultState"))
(define search-started (Concept "SearchStarted"))

(define aiml-replies (Anchor "AIMLReplies"))
(define no-aiml-reply (Concept "NoAIMLReply"))
(define aiml-search (Anchor "AIMLSearch"))
(State aiml-replies default-state)
(State aiml-search default-state)

(define fuzzy-replies (Anchor "FuzzyReplies"))
(define no-fuzzy-reply (Concept "NoFuzzyReply"))
(define fuzzy-search (Anchor "FuzzySearch"))
(State fuzzy-replies default-state)
(State fuzzy-search default-state)

(define fuzzy-answers (Anchor "FuzzyAnswers"))
(define no-fuzzy-answers (Concept "NoFuzzyAnswers"))
(define fuzzy-qa-search (Anchor "FuzzyQASearch"))
(State fuzzy-answers default-state)
(State fuzzy-qa-search default-state)

;-------------------------------------------------------------------------------
; Define the demands

(define sociality (psi-demand "Sociality" .8))

;-------------------------------------------------------------------------------

; Load the available contexts
(load "contexts.scm")

; Load the available actions
(load "actions.scm")

; Load the psi-rules
(load "psi-rules.scm")

; Load the utilities
(load "utils.scm")

; Run OpenPsi if it's not already running
(if (not (psi-running?))
    (psi-run)
)
