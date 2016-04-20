; Copyright (C) 2016 OpenCog Foundation
; This is a basic example of how an OpenPsi driven dialogue system may look like

; Steps to run:
; 1. (add-to-load-path "/absolute/path/to/opencog/opencog")
;    e.g. (add-to-load-path "/opencog/opencog")
; 2. (load-from-path "../opencog/openpsi/main.scm")
; 3. (load "../examples/openpsi/chat.scm")
; 4. Use (chat) function to talk to it
;    e.g. (chat "Are you conscious?")

(add-to-load-path "/opencog/opencog")
(load-from-path "../opencog/openpsi/main.scm")

;-------------------------------------------------------------------------------

(define chat-rule (ConceptNode "OpenPsi: Chat Rule"))
(define no-new-input-utterance (ConceptNode "No New Input Utterance"))
(define new-input-utterance (AnchorNode "New Input Utterance"))
(StateLink new-input-utterance no-new-input-utterance)
(define (chat utt) (StateLink new-input-utterance (car (nlp-parse utt))))

;-------------------------------------------------------------------------------
; Helper functions

(define (is-chat-rule? rule)
    (let ((nodes (cog-chase-link 'MemberLink 'ConceptNode rule)))
        (if (member chat-rule nodes)
            #t
            #f
        ))
)

;-------------------------------------------------------------------------------
; Define the demands and their default values

(define sociality (psi-demand "Sociality" .8))
(define please-user (psi-demand "PleaseUser" .8))
(define learn-stuff (psi-demand "LearnStuff" .8))
(define humor (psi-demand "Humor" .8))

;-------------------------------------------------------------------------------
; Define an action selector

(define (psi-action-selector-chat)
    (let* ((max-score .5)
           (best-match "")
           (satisfied-rules '())
           (all-demands (cog-outgoing-set (psi-get-demands-all)))
           (all-rules (append-map psi-get-rules all-demands))
          )
        (map (lambda (r)
            (if (is-chat-rule? r)
                (let ((score (tv-mean (psi-satisfiable? r))))
                    (if (> score max-score)
                        (begin (set! max-score score) (set! best-match r))
                    )
                )
                (if (equal? (stv 1 1) (psi-satisfiable? r))
                    (set! satisfied-rules (append satisfied-rules (list r)))
                )
            ))
        all-rules)

        (if (cog-atom? best-match)
            (if (> (length satisfied-rules) 0)
                (SetLink (append satisfied-rules (list best-match)))
                best-match
            )            
            (SetLink satisfied-rules)
        )
    )
)

(DefineLink
    (DefinedSchemaNode "psi-action-selector-chat")
    (ExecutionOutputLink
        (GroundedSchemaNode "scm: psi-action-selector-chat")
        (ListLink)
    )
)

(define dsn (DefinedSchemaNode "psi-action-selector-chat"))
(psi-action-selector-set! dsn)

;-------------------------------------------------------------------------------
; Those grounded Scheme functions for the chatbot

(define (check-demand demand value)
    (if (< (tv-mean (cog-tv demand)) (string->number (cog-name value)))
        (stv 1 1)
        (stv 0 1)
    )
)

(define (do-fuzzy-match rule-utt)
    (let ((input-utt (car (cog-outgoing-set (cog-execute! (Get (State new-input-utterance (Variable "$x"))))))))
        (if (equal? input-utt no-new-input-utterance)
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
    (display (sent-get-words-in-order utterance))
    (newline)
    (State new-input-utterance no-new-input-utterance)
)

(define (psi-update-demand-chat-min demand weight)
    (psi-demand-value-minimize demand weight)
    (stv 1 1)
)

(define (psi-update-demand-chat-max demand weight)
    (psi-demand-value-maximize demand weight)
    (stv 1 1)
)

;-------------------------------------------------------------------------------
; Add psi-rules

;----------
; These rules change the demand values whenever there is an input utterance

(psi-rule
    (list (NotLink (EqualLink (SetLink no-new-input-utterance) (GetLink (StateLink new-input-utterance (VariableNode "$x"))))))
    (ListLink)
    (EvaluationLink (GroundedPredicateNode "scm:psi-update-demand-chat-min") (ListLink sociality (NumberNode "40")))
    (stv 1 1)
    sociality
)
(psi-rule
    (list (NotLink (EqualLink (SetLink no-new-input-utterance) (GetLink (StateLink new-input-utterance (VariableNode "$x"))))))
    (ListLink)
    (EvaluationLink (GroundedPredicateNode "scm:psi-update-demand-chat-min") (ListLink please-user (NumberNode "40")))
    (stv 1 1)
    please-user
)

;----------
; Input: Are you conscious?
; Output: I'm as conscious as you are, meat machine!

(define in_utt (nlp-parse "Are you conscious?"))
(define out_utt (nlp-parse "I'm as conscious as you are, meat machine!"))
(MemberLink
    (psi-rule
        (list (EvaluationLink (GroundedPredicateNode "scm:check-demand") (ListLink sociality (NumberNode "0.6")))
              (EvaluationLink (GroundedPredicateNode "scm:do-fuzzy-match") (ListLink in_utt)))
        (ExecutionOutputLink (GroundedSchemaNode "scm:say") (ListLink out_utt))
        (EvaluationLink (GroundedPredicateNode "scm:psi-update-demand-chat-max") (ListLink sociality (NumberNode "20")))
        (stv 1 1)
        sociality
    )
    chat-rule
)
(MemberLink
    (psi-rule
        (list (EvaluationLink (GroundedPredicateNode "scm:check-demand") (ListLink please-user (NumberNode "0.6")))
              (EvaluationLink (GroundedPredicateNode "scm:do-fuzzy-match") (ListLink in_utt)))
        (ExecutionOutputLink (GroundedSchemaNode "scm:say") (ListLink out_utt))
        (EvaluationLink (GroundedPredicateNode "scm:psi-update-demand-chat-max") (ListLink please-user (NumberNode "15")))
        (stv 1 1)
        please-user
    )
    chat-rule
)
(MemberLink
    (psi-rule
        (list (EvaluationLink (GroundedPredicateNode "scm:check-demand") (ListLink humor (NumberNode "0.6")))
              (EvaluationLink (GroundedPredicateNode "scm:do-fuzzy-match") (ListLink in_utt)))
        (ExecutionOutputLink (GroundedSchemaNode "scm:say") (ListLink out_utt))
        (EvaluationLink (GroundedPredicateNode "scm:psi-update-demand-chat-max") (ListLink humor (NumberNode "30")))
        (stv 1 1)
        humor
    )
    chat-rule
)

;----------
; Input: What do you call an alligator in a vest?
; Output: An Investigator.

(define in_utt (nlp-parse "What do you call an alligator in a vest?"))
(define out_utt (nlp-parse "An Investigator."))
(MemberLink
    (psi-rule
        (list (EvaluationLink (GroundedPredicateNode "scm:check-demand") (ListLink sociality (NumberNode "0.6")))
              (EvaluationLink (GroundedPredicateNode "scm:do-fuzzy-match") (ListLink in_utt)))
        (ExecutionOutputLink (GroundedSchemaNode "scm:say") (ListLink out_utt))
        (EvaluationLink (GroundedPredicateNode "scm:psi-update-demand-chat-max") (ListLink sociality (NumberNode "30")))
        (stv 1 1)
        sociality
    )
    chat-rule
)
(MemberLink
    (psi-rule
        (list (EvaluationLink (GroundedPredicateNode "scm:check-demand") (ListLink please-user (NumberNode "0.6")))
              (EvaluationLink (GroundedPredicateNode "scm:do-fuzzy-match") (ListLink in_utt)))
        (ExecutionOutputLink (GroundedSchemaNode "scm:say") (ListLink out_utt))
        (EvaluationLink (GroundedPredicateNode "scm:psi-update-demand-chat-max") (ListLink please-user (NumberNode "10")))
        (stv 1 1)
        please-user
    )
    chat-rule
)
(MemberLink
    (psi-rule
        (list (EvaluationLink (GroundedPredicateNode "scm:check-demand") (ListLink humor (NumberNode "0.6")))
              (EvaluationLink (GroundedPredicateNode "scm:do-fuzzy-match") (ListLink in_utt)))
        (ExecutionOutputLink (GroundedSchemaNode "scm:say") (ListLink out_utt))
        (EvaluationLink (GroundedPredicateNode "scm:psi-update-demand-chat-max") (ListLink humor (NumberNode "40")))
        (stv 1 1)
        humor
    )
    chat-rule
)

;----------
; Input: How are you?
; Output: I am great, thanks.

(define in_utt (nlp-parse "How are you?"))
(define out_utt (nlp-parse "I am great, thanks."))
(MemberLink
    (psi-rule
        (list (EvaluationLink (GroundedPredicateNode "scm:check-demand") (ListLink sociality (NumberNode "0.6")))
              (EvaluationLink (GroundedPredicateNode "scm:do-fuzzy-match") (ListLink in_utt)))
        (ExecutionOutputLink (GroundedSchemaNode "scm:say") (ListLink out_utt))
        (EvaluationLink (GroundedPredicateNode "scm:psi-update-demand-chat-max") (ListLink sociality (NumberNode "20")))
        (stv 1 1)
        sociality
    )
    chat-rule
)
(MemberLink
    (psi-rule
        (list (EvaluationLink (GroundedPredicateNode "scm:check-demand") (ListLink please-user (NumberNode "0.6")))
              (EvaluationLink (GroundedPredicateNode "scm:do-fuzzy-match") (ListLink in_utt)))
        (ExecutionOutputLink (GroundedSchemaNode "scm:say") (ListLink out_utt))
        (EvaluationLink (GroundedPredicateNode "scm:psi-update-demand-chat-max") (ListLink please-user (NumberNode "15")))
        (stv 1 1)
        please-user
    )
    chat-rule
)

;----------
; Input: There is a monster.
; Output: What is a monster?

(define in_utt (nlp-parse "There is a monster."))
(define out_utt (nlp-parse "What is a monster?"))
(MemberLink
    (psi-rule
        (list (EvaluationLink (GroundedPredicateNode "scm:check-demand") (ListLink sociality (NumberNode "0.6")))
              (EvaluationLink (GroundedPredicateNode "scm:do-fuzzy-match") (ListLink in_utt)))
        (ExecutionOutputLink (GroundedSchemaNode "scm:say") (ListLink out_utt))
        (EvaluationLink (GroundedPredicateNode "scm:psi-update-demand-chat-max") (ListLink sociality (NumberNode "20")))
        (stv 1 1)
        sociality
    )
    chat-rule
)
(MemberLink
    (psi-rule
        (list (EvaluationLink (GroundedPredicateNode "scm:check-demand") (ListLink learn-stuff (NumberNode "0.6")))
              (EvaluationLink (GroundedPredicateNode "scm:do-fuzzy-match") (ListLink in_utt)))
        (ExecutionOutputLink (GroundedSchemaNode "scm:say") (ListLink out_utt))
        (EvaluationLink (GroundedPredicateNode "scm:psi-update-demand-chat-max") (ListLink learn-stuff (NumberNode "35")))
        (stv 1 1)
        learn-stuff
    )
    chat-rule
)

;-------------------------------------------------------------------------------
; Run OpenPsi

(psi-run)

