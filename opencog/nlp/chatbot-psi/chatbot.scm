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
    (let* ((sent-node (car (nlp-parse utterance)))
           (speech-act (cog-name (car (sentence-get-utterance-type sent-node)))))
        (State input-utterance
            (Reference
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
                sent-node
            )
        )
        (State
            input-utterance-time
            (Time (current-time))
        )
        ; TODO: Check the speech act and feed to external sources
        (if (equal? speech-act "InterrogativeSpeechAct")
            (display "Should send to external source for finding answers...")
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

; Run OpenPsi
(psi-run)
