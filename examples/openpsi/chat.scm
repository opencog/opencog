; Copyright (C) 2016 OpenCog Foundation
; This is a simple example of how an OpenPsi driven dialogue system may look like

; Steps to run:
; 1. Load this example, e.g.
;    (load "../examples/openpsi/chat.scm")
; 2. Use (chat) function to talk to it, e.g.
;    (chat "Are you conscious?")

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
; Keep track of the chat-state so that the psi-rules can make use of them

(define psi-perceive #t)
(define (psi-perceive-on) (set! psi-perceive #t))
(define (psi-perceive-off) (set! psi-perceive #f))
(define perception-rule (Concept (string-append (psi-prefix-str) "Perception Rule")))

(define psi-chat #t)
(define (psi-chat-on) (set! psi-chat #t))
(define (psi-chat-off) (set! psi-chat #f))
(define chat-rule (Concept (string-append (psi-prefix-str) "Chat Rule")))

(define input-utterance (Anchor "Input Utterance"))
(define no-input-utterance (Concept "No Input Utterance"))
(State input-utterance no-input-utterance)

(define (chat utt) (State input-utterance (car (nlp-parse utt))) (newline))

;-------------------------------------------------------------------------------
; Define the demands with their default values

(define sociality (psi-demand "Sociality" .8))

;-------------------------------------------------------------------------------
; Define and set an action selector

(define (psi-action-selector-chat)
    (define (is-chat-rule? rule)
        (let ((nodes (cog-chase-link 'MemberLink 'ConceptNode rule)))
            (if (member chat-rule nodes)
                #t
                #f
            ))
    )
    (let* ((max-score .5)
           (best-matches '())
           (satisfied-rules '())
           (all-demands (cog-outgoing-set (psi-get-all-demands)))
           (all-rules (append-map psi-get-rules all-demands))
          )
        (map (lambda (r)
            (if (is-chat-rule? r)
                (let ((score (tv-mean (psi-satisfiable? r))))
                    (if (> score max-score)
                        (begin (set! max-score score) (set! best-matches (list r)))
                    )
                    (if (= score max-score)
                        (append! best-matches (list r))
                    )
                )
                ; For other types of rules, just check if they are satisfiable
                ; TODO: Ideally for non-chat rules, they should be handled
                ;       by the default action selector or other acion selectors
                ;       already defined in the system
                (if (equal? (stv 1 1) (psi-satisfiable? r))
                    (set! satisfied-rules (append satisfied-rules (list r)))
                )
            ))
        all-rules)

        (if (> (length best-matches) 0)
            (if (> (length satisfied-rules) 0)
                (Set (append satisfied-rules (list (list-ref best-matches (random (length best-matches))))))
                (list-ref best-matches (random (length best-matches)))
            )            
            (Set satisfied-rules)
        )
    )
)

(psi-action-selector-set!
    (psi-add-action-selector
        (ExecutionOutput (GroundedSchema "scm: psi-action-selector-chat") (List))
        "chat"))

;-------------------------------------------------------------------------------
; Scheme functions that will be called by the rules

(define (check-demand demand value)
    (if (< (tv-mean (cog-tv demand)) (string->number (cog-name value)))
        (stv 1 1)
        (stv 0 1)
    )
)

(define (do-fuzzy-match rule-utt)
    (let ((input-utt (car (cog-outgoing-set (cog-execute! (Get (State input-utterance (Variable "$x"))))))))
        (if (equal? input-utt no-input-utterance)
            (stv 0 1)
            (let* ((input-r2l (get-r2l-set-of-sent input-utt))
                   (rule-r2l (get-r2l-set-of-sent rule-utt))
                   (score (string->number (cog-name (nlp-fuzzy-compare input-r2l rule-r2l))))
                  )
                (stv score 1)
            )
        )
    )
)

(define (say utterance)
    (display (list (map word-inst-get-word-str (cdr (car (sent-get-words-in-order utterance))))))
    (newline)
    (State input-utterance no-input-utterance)
)

;-------------------------------------------------------------------------------
; Add and tag new psi-rules

;----------
; These rules change the demand values whenever there is an input utterance

(Member
    (psi-rule
        (list (Not (Equal (Set no-input-utterance) (Get (State input-utterance (Variable "$x"))))))
        (ExecutionOutput (GroundedSchema "scm:psi-chat-on") (List))
        (Evaluation (GroundedPredicate "scm:psi-demand-value-minimize") (List sociality (Number "10")))
        (stv 1 1)
        sociality
    )
    perception-rule
)

;----------
; Input: Are you conscious?

(define in_utt (nlp-parse "Are you conscious?"))
(define out_utt_1 (nlp-parse "I'm as conscious as you are, meat machine!"))
(define out_utt_2 (nlp-parse "Yes I am."))
(define out_utt_3 (nlp-parse "What do you think?"))
(define out_utt_4 (nlp-parse "Of course."))
(Member
    (psi-rule
        (list (Evaluation (GroundedPredicate "scm:do-fuzzy-match") (List in_utt)))
        (ExecutionOutput (GroundedSchema "scm:say") (List out_utt_1))
        (Evaluation (GroundedPredicate "scm:psi-demand-value-maximize") (List sociality (Number "100")))
        (stv 1 1)
        sociality
    )
    chat-rule
)
(Member
    (psi-rule
        (list (Evaluation (GroundedPredicate "scm:do-fuzzy-match") (List in_utt)))
        (ExecutionOutput (GroundedSchema "scm:say") (List out_utt_2))
        (Evaluation (GroundedPredicate "scm:psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    chat-rule
)
(Member
    (psi-rule
        (list (Evaluation (GroundedPredicate "scm:do-fuzzy-match") (List in_utt)))
        (ExecutionOutput (GroundedSchema "scm:say") (List out_utt_3))
        (Evaluation (GroundedPredicate "scm:psi-demand-value-maximize") (List sociality (Number "80")))
        (stv .8 1)
        sociality
    )
    chat-rule
)
(Member
    (psi-rule
        (list (Evaluation (GroundedPredicate "scm:do-fuzzy-match") (List in_utt)))
        (ExecutionOutput (GroundedSchema "scm:say") (List out_utt_4))
        (Evaluation (GroundedPredicate "scm:psi-demand-value-maximize") (List sociality (Number "90")))
        (stv .9 1)
        sociality
    )
    chat-rule
)

;----------
; The default reply if there is no matching rule in the system

(Member
    (psi-rule
        (list (Evaluation (GroundedPredicate "scm:check-demand") (List sociality (Number "0.1"))))
        (ExecutionOutput (GroundedSchema "scm:say") (List (nlp-parse "Sorry I don't understand.")))
        (Evaluation (GroundedPredicate "scm:psi-demand-value-maximize") (List sociality (Number "100")))
        (stv 1 1)
        sociality
    )
    chat-rule
)

;-------------------------------------------------------------------------------
; Run OpenPsi

(psi-run)
