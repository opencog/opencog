; Copyright (C) 2016 OpenCog Foundation
; This is a simple example of how an OpenPsi driven dialogue system may look like

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
; Steps to run:
; 1. Make sure the above relex-server-host is set correctly
; 2. Load this example in Guile, e.g.
;    (load "../examples/openpsi/chat.scm")
; 3. Use (chat) function to talk to it, e.g.
;    (chat "Are you conscious?")

;-------------------------------------------------------------------------------
; Keep track of the states

(define input-utterance (Anchor "Input Utterance"))
(define no-input-utterance (Concept "No Input Utterance"))
(State input-utterance no-input-utterance)

;-------------------------------------------------------------------------------
; Types of rules we have

; Should always on
(define psi-perception #t)
(define (psi-perception-on) (set! psi-perception #t))
(define (psi-perception-off) (set! psi-perception #f))
(define perception-rule (Concept (string-append (psi-prefix-str) "Perception Rule")))

; On mostly when someone talks to the robot
(define psi-chat #f)
(define (psi-chat-on) (set! psi-chat #t))
(define (psi-chat-off) (set! psi-chat #f))
(define chat-rule (Concept (string-append (psi-prefix-str) "Chat Rule")))

;-------------------------------------------------------------------------------
; Utilities

(define (get-rules tag) (cog-chase-link 'MemberLink 'ImplicationLink tag))
(define (chat utt) (State input-utterance (car (nlp-parse utt))) (newline))

;-------------------------------------------------------------------------------
; Define the demands with their default values

(define sociality (psi-demand "Sociality" .8))
(define humor (psi-demand "Humor" .5))

;-------------------------------------------------------------------------------
; Define and set an action selector

(define (psi-action-selector-chat)
    (let ((satisfied-rules '()))
        (if psi-perception
            (map
                (lambda (r)
                    (if (equal? (stv 1 1) (psi-satisfiable? r))
                        (set! satisfied-rules (append (list r)))
                    )
                )
                (get-rules perception-rule)
            )
        )

        (if psi-chat
            ; Find the best matches
            (let ((max-score .5)
                  (best-matches '()))
                (map
                    (lambda (r)
                        (let ((score (tv-mean (psi-satisfiable? r))))
                            (if (> score max-score)
                                (begin
                                    (set! max-score score)
                                    (set! best-matches (list r))
                                )
                            )
                            (if (= score max-score)
                                (append! best-matches (list r))
                            )
                        )
                    )
                    ; TODO: Need to reduce the search space
                    (get-rules chat-rule)
                )

                (psi-chat-off)

                ; Randomly pick one of them
                ; TODO: Should have a higher probability to pick the one
                ;       that can satisfy most of the demands, or something similar?
                (if (> (length best-matches) 0)
                    (set! satisfied-rules (append satisfied-rules
                        (list (list-ref best-matches (random (length best-matches))))))
                )
            )
        )
;(if (> (length satisfied-rules) 1) (begin (display "\n=====================================\npsi-action-selctor-chat chosen rules:\n")(display satisfied-rules)))
        ; Return all the satisfied rules
        (if (> (length satisfied-rules) 0)
            (Set satisfied-rules)
            (Set)
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
(define out_utt_5 (nlp-parse "Let me check."))
(Member
    (psi-rule
        (list (Evaluation (GroundedPredicate "scm:do-fuzzy-match") (List in_utt)))
        (ExecutionOutput (GroundedSchema "scm:say") (List out_utt_1))
        (Evaluation (GroundedPredicate "scm:psi-demand-value-maximize") (List sociality (Number "80")))
        (stv .8 1)
        sociality
    )
    chat-rule
)
(Member
    (psi-rule
        (list (Evaluation (GroundedPredicate "scm:do-fuzzy-match") (List in_utt)))
        (ExecutionOutput (GroundedSchema "scm:say") (List out_utt_1))
        (Evaluation (GroundedPredicate "scm:psi-demand-value-maximize") (List humor (Number "70")))
        (stv .7 1)
        humor
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
(Member
    (psi-rule
        (list (Evaluation (GroundedPredicate "scm:do-fuzzy-match") (List in_utt)))
        (ExecutionOutput (GroundedSchema "scm:say") (List out_utt_5))
        (Evaluation (GroundedPredicate "scm:psi-demand-value-maximize") (List sociality (Number "75")))
        (stv .75 1)
        sociality
    )
    chat-rule
)
(Member
    (psi-rule
        (list (Evaluation (GroundedPredicate "scm:do-fuzzy-match") (List in_utt)))
        (ExecutionOutput (GroundedSchema "scm:say") (List out_utt_5))
        (Evaluation (GroundedPredicate "scm:psi-demand-value-maximize") (List humor (Number "80")))
        (stv .8 1)
        humor
    )
    chat-rule
)

;----------
; The default reply if there is no matching rule in the system

(Member
    (psi-rule
        (list
            (Not (Equal (Set no-input-utterance) (Get (State input-utterance (Variable "$x")))))
            (Evaluation (GroundedPredicate "scm:check-demand") (List sociality (Number "0.1")))
        )
        (ExecutionOutput (GroundedSchema "scm:say") (List (nlp-parse "I don't understand.")))
        (Evaluation (GroundedPredicate "scm:psi-demand-value-maximize") (List sociality (Number "100")))
        (stv 1 1)
        sociality
    )
    chat-rule
)

;-------------------------------------------------------------------------------
; Run OpenPsi

;(psi-run)
