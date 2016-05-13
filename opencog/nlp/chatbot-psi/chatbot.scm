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
    (let ((sent-node (car (nlp-parse utterance))))
        (State input-utterance (List
            (append-map
                (lambda (w)
                    (if (not (string-prefix? "LEFT-WALL" (cog-name w)))
                        (cog-chase-link 'ReferenceLink 'WordNode w)
                        '()
                    )
                )
                (car (sent-get-words-in-order sent-node))
            )
        ))
    )
    (newline)
)

;-------------------------------------------------------------------------------
; Keep track of the states

(define-public input-utterance (Anchor "InputUtterance"))
(define-public no-input-utterance (Concept "NoInputUtterance"))
(State input-utterance no-input-utterance)

;-------------------------------------------------------------------------------
; Define the demands

(define-public sociality (psi-demand "Sociality" .8))

;-------------------------------------------------------------------------------

; Run OpenPsi
(psi-run)
