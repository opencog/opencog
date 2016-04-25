; Copyright (C) 2016 OpenCog Foundation
; This is a simple example of how an OpenPsi driven dialogue system may look like

; Steps to run:
; 1. (add-to-load-path "/absolute/path/to/opencog/opencog")
;    e.g. (add-to-load-path "/opencog/opencog")
; 2. (load "../examples/openpsi/chat.scm")
; 3. Use (chat) function to talk to it
;    e.g. (chat "Are you conscious?")

(load-from-path "../opencog/openpsi/main.scm")

;-------------------------------------------------------------------------------
; Keep track of the chat-state so that the psi-rules can make use of them

(define chat-rule (ConceptNode "OpenPsi: Chat Rule"))
(define no-new-input-utterance (ConceptNode "No New Input Utterance"))
(define new-input-utterance (AnchorNode "New Input Utterance"))
(StateLink new-input-utterance no-new-input-utterance)
(define (chat utt) (StateLink new-input-utterance (car (nlp-parse utt))) (newline))

;-------------------------------------------------------------------------------
; Define the demands with their default values

(define sociality (psi-demand "Sociality" .8))
(define please-user (psi-demand "PleaseUser" .8))
(define learn-stuff (psi-demand "LearnStuff" .8))
(define humor (psi-demand "Humor" .8))

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
                (SetLink (append satisfied-rules (list (list-ref best-matches (random (length best-matches))))))
                (list-ref best-matches (random (length best-matches)))
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
; Scheme functions that will be called by the rules

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
    (display (list (map word-inst-get-word-str (cdr (car (sent-get-words-in-order utterance))))))
    (newline)
    (State new-input-utterance no-new-input-utterance)
)

;-------------------------------------------------------------------------------
; Add and tag new psi-rules

;----------
; These rules change the demand values whenever there is an input utterance

(psi-rule
    (list (NotLink (EqualLink (SetLink no-new-input-utterance) (GetLink (StateLink new-input-utterance (VariableNode "$x"))))))
    (ListLink)
    (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-minimize") (ListLink sociality (NumberNode "10")))
    (stv 1 1)
    sociality
)
(psi-rule
    (list (NotLink (EqualLink (SetLink no-new-input-utterance) (GetLink (StateLink new-input-utterance (VariableNode "$x"))))))
    (ListLink)
    (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-minimize") (ListLink please-user (NumberNode "10")))
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
        (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-maximize") (ListLink sociality (NumberNode "30")))
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
        (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-maximize") (ListLink please-user (NumberNode "20")))
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
        (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-maximize") (ListLink humor (NumberNode "30")))
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
        (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-maximize") (ListLink sociality (NumberNode "30")))
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
        (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-maximize") (ListLink please-user (NumberNode "30")))
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
        (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-maximize") (ListLink humor (NumberNode "40")))
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
        (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-maximize") (ListLink sociality (NumberNode "40")))
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
        (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-maximize") (ListLink please-user (NumberNode "20")))
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
        (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-maximize") (ListLink sociality (NumberNode "30")))
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
        (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-maximize") (ListLink learn-stuff (NumberNode "50")))
        (stv 1 1)
        learn-stuff
    )
    chat-rule
)

;----------
; The default reply if there is no matching rule in the system

(MemberLink
    (psi-rule
        (list (EvaluationLink (GroundedPredicateNode "scm:check-demand") (ListLink sociality (NumberNode "0.1"))))
        (ExecutionOutputLink (GroundedSchemaNode "scm:say") (ListLink (nlp-parse "Sorry I don't understand.")))
        (EvaluationLink (GroundedPredicateNode "scm:psi-demand-value-maximize") (ListLink sociality (NumberNode "100")))
        (stv 1 1)
        sociality
    )
    chat-rule
)
;-------------------------------------------------------------------------------
; Run OpenPsi

(psi-run)

