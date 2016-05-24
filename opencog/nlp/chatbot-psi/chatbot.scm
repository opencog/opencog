(use-modules (opencog)
             (opencog nlp)
             (opencog nlp relex2logic)
             (opencog nlp fuzzy)
             (opencog nlp chatbot)
             (opencog exec)
             (opencog openpsi))

(load-r2l-rulebase)
(set! relex-server-host "172.17.0.2")

;-------------------------------------------------------------------------------
; Schema function for chatting

(define (chat utterance)
    (define (get-words-list sent-node)
        (List
            (append-map
                (lambda (w)
                    (if (not (string-prefix? "LEFT-WALL" (cog-name w)))
                        (cog-chase-link 'ReferenceLink 'WordNode w)
                        '()
                    )
                )
                (car (sent-get-words-in-order sent-node))
            )
        )
    )

    (let* ((sent-node (car (nlp-parse utterance)))
           (speech-act (cog-name (car (sentence-get-utterance-type sent-node))))
           (list-of-words (get-words-list sent-node)))

        (State input-utterance
            (Reference
                list-of-words
                sent-node
            )
        )

        ; Record the time
        (State
            input-utterance-time
            (Time (current-time))
        )

        ; Start searching for related canned rules, if any
        ; TODO: Check if the word is actually in the psi-context as well
        (begin-thread
            (define (no-punctuation w)
                (not (equal? (cog-name
                    (car (cog-chase-link 'PartOfSpeechLink 'DefinedLinguisticConceptNode
                        (car (cog-chase-link 'ReferenceLink 'WordInstanceNode w)))))
                    "punctuation"
                ))
            )
            (let* ((filtered-words (filter no-punctuation (cog-outgoing-set list-of-words)))
                   (roots (delete-duplicates (append-map cog-get-root filtered-words)))
                   (rules '()))
                (map
                    (lambda (m)
                        (if (equal? (cog-type m) 'MemberLink)
                            (if (equal? (cadr (cog-outgoing-set m)) canned-rule)
                                (set! rules (append rules (list (car (cog-outgoing-set m)))))
                            )
                        )
                    )
                    roots
                )
                (State canned-rules (List rules))
            )
        )
    )
    (newline)
)

;-------------------------------------------------------------------------------
; Keep track of the states

(define-public input-utterance-time (Anchor "InputUtteranceTime"))
(define-public input-utterance (Anchor "InputUtterance"))
(define-public no-input-utterance (Concept "NoInputUtterance"))
(State input-utterance no-input-utterance)

(define-public default-state (Anchor "DefaultState"))

(define-public canned-rules (Anchor "CannedRules"))
(define-public no-canned-rules (Anchor "NoCannedRules"))
(State canned-rules default-state)

(define-public fuzzy-answers (Anchor "FuzzyAnswers"))
(define-public no-fuzzy-answers (Anchor "NoFuzzyAnswers"))
(State fuzzy-answers default-state)

;-------------------------------------------------------------------------------
; Define the demands

(define-public sociality (psi-demand "Sociality" .8))

;-------------------------------------------------------------------------------

; Load the available contexts
(load "contexts.scm")

; Load the available actions
(load "actions.scm")

; Load the psi-rules
(load "psi-rules.scm")

; Load the action selector
(load "action-selector.scm")

; Load the utilities
(load "utils.scm")

; Run OpenPsi
(psi-run)
