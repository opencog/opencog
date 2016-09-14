; Copyright (C) 2016 OpenCog Foundation

; --------------------------------------------------------------
; If you want to run this in guile wihout installing,
; 1. run cmake in the build directory
; 2. run  (add-to-load-path "/absolute/path/to/build/opencog/scm")
; 3. (use-modules (opencog) (opencog openpsi))

; NOTE:
; 1. The context of the rules are created so as to test possible generic
;    atomese patterns during application development, thus the semantics might
;    not make sense.
; 2. The numbering of the demands, context, action, and goals are used for
;    differentiating and are not necessarily related with the number of tests.
; --------------------------------------------------------------
(define context-1
    (list
       (ListLink
            (VariableNode "x1")
            (VariableNode "y1")
            (ConceptNode "Required constant for DualLink-1")
            (VariableNode "z1"))
        (InheritanceLink
            (VariableNode "x1")
            (VariableNode "z1"))
        (EqualLink
            (VariableNode "x1")
            (VariableNode "y1"))
        ))

(define action-1
    (EvaluationLink
        (GroundedPredicate "scm: act-1")
        (ListLink (Variable "$abc"))))

(define (act-1 groundings)
    (ConceptNode "act-1")
    (stv 1 1)
)

(define demand-1 (psi-demand  "demand-1" .87))

(define goal-1
    (EvaluationLink
        (GroundedPredicateNode "scm: test-update-tv")
        (ListLink
            demand-1
            (NumberNode .5))))

(define (test-update-tv node strength)
    (cog-set-tv! node
        (stv (string->number (cog-name strength)) (tv-conf (cog-tv node))))
    (stv 1 1)
)

(define (rule-1) (psi-rule context-1 action-1 goal-1 (stv 1 1) demand-1))

(define (groundable-content-1)
    (list
        (ListLink
            (NumberNode 1)
            (NumberNode 1)
            (ConceptNode "Required constant for DualLink-1")
            (PredicateNode "z"))
        (InheritanceLink
            (NumberNode 1)
            (PredicateNode "z")))
)

; --------------------------------------------------------------
(define context-2
    (list ; They are in a list so as to simplify removal.
       (ListLink
            (VariableNode "x2")
            (ConceptNode "Required constant for DualLink-2")
            (VariableNode "z2"))
        (InheritanceLink
            (VariableNode "x2")
            (VariableNode "z2"))
        (NotLink (EqualLink
            (VariableNode "x2")
            (VariableNode "z2")))
        ))

(define action-2
    (EvaluationLink
        (GroundedPredicate "scm: act-2")
        (ListLink (Variable "$abc"))))

(define (act-2 groundings)
    (ConceptNode "act-2")
    (stv 1 1)
)

(define demand-2 (psi-demand  "demand-2" .87))

(define goal-2
    (EvaluationLink
        (GroundedPredicateNode "scm: test-update-tv")
        (ListLink
            demand-2
            (NumberNode .5))))

(define (rule-2) (psi-rule context-2 action-2 goal-2 (stv 1 1) demand-2))

(define (groundable-content-2)
    (list ; They are in a list so as to simplify removal.
        (ListLink
            (NumberNode 1)
            (ConceptNode "Required constant for DualLink-2")
            (NumberNode 2))
        (InheritanceLink
            (NumberNode 1)
            (NumberNode 2)))
)

(define (rule-3) (psi-rule context-2 action-2 goal-1 (stv 1 1) demand-1))

; --------------------------------------------------------------
; Helper functions for `OpenPsiUTest::test_psi_related_goals`
(define (test_psi_related_goals_1)
    (equal? goal-1 (car (psi-related-goals action-1)))
)

(define (test_psi_related_goals_2)
    (if (and
            (member goal-1 (psi-related-goals action-2))
            (member goal-2 (psi-related-goals action-2)))
        #t
        #f
    )
)

; --------------------------------------------------------------
; Helper functions for `OpenPsiUTest::test_psi_step_*
(define (setup_test_psi_step_1)
    ; Load rules
    (rule-1)
    (rule-2)
    (rule-3)
    ; Make one step
    (psi-step)
)

(define (demand-value demand-node)
"
  Returns the strength of the demand-node to two decimal places.
"
    (/ (round (* 100 (tv-mean (cog-tv demand-node))) ) 100)
)

(define (setup_test_psi_step_2)
    ; Load rules
    (rule-1)
    (rule-2)
    (rule-3)
    ; Load groundable contents for satisfying rule-1 only
    (groundable-content-1)
    ; Make one step
    (psi-step)
)

(define (test_psi_step_2_1) (cog-node? (cog-node 'ConceptNode "act-1")))

(define (setup_test_psi_step_3)
    ; Load rules
    (rule-1)
    (rule-2)
    (rule-3)
    ; Load groundable contents for satisfying rule-2 or rule 3
    (groundable-content-2)
    ; Make one step
    (psi-step)
)

(define (test_psi_step_3_1) (cog-node? (cog-node 'ConceptNode "act-2")))

; --------------------------------------------------------------
; Helper functions for `OpenPsiUTest::test_psi_get_dual_rules`
(define demand-4 (psi-demand  "demand-4" .87))

(define action-4
    (EvaluationLink
        (GroundedPredicate "scm: act-4")
        (ListLink (Variable "$abc"))))

(define (act-4 groundings)
    (ConceptNode "act-4")
    (stv 1 1)
)

(define goal-4
    (EvaluationLink
        (GroundedPredicateNode "scm: test-update-tv")
        (ListLink
            demand-4
            (NumberNode .5))))


(define (groundable-content-4)
    (list ; They are in a list so as to simplify removal.
        (ListLink
            (NumberNode 1)
            (NumberNode 2))
        (InheritanceLink
            (NumberNode 1)
            (NumberNode 2)))
)

; (rule-4) & (rule-5) are for usage with DualLink version of action-selection
; i.e. psi-satisfiable? should use DualLink
(define (rule-4)
    (psi-rule (groundable-content-4) action-4 goal-4 (stv 1 1) demand-4))

; --------------------------------------------------------------
; This doesn't have any groudings and as such is a rule of the
; evaluatable form.
(define (rule-6)
    (psi-rule (list (True)) action-6 goal-4 (stv 1 1) demand-4)
)

(define action-6
    (EvaluationLink
        (GroundedPredicate "scm: act-6")
        (ListLink)))

(define (act-6)
    (ConceptNode "act-6")
    (stv 1 1)
)

(define (setup_test_psi_step_4)
    ; Load rule
    (rule-6)
    ; Make one step
    (psi-step)
)

(define (test_psi_step_4) (cog-node? (cog-node 'ConceptNode "act-6")))

; --------------------------------------------------------------
;(define demand-5 (psi-demand  "demand-5" .87))

;(define (rule-5)
;    (psi-rule
;        (list context-1 (groundable-content-4))
;        action-1 goal-2 (stv 1 1) demand-5)
;)

;(define (test_psi_get_dual_rules_1_1)
;    (equal? (car (psi-get-dual-rules (car (groundable-content-4)))) (rule-4))
;)
;
;(define (test_psi_get_dual_rules_1_2)
;    (equal? (car (psi-get-dual-rules (cadr (groundable-content-4)))) (rule-4))
;)
;
;(define (test_psi_get_dual_rules_2_1)
;    (length (psi-get-dual-rules (car (groundable-content-1))))
;)
;
;(define (test_psi_get_dual_rules_2_2)
;    (if (and
;            (member (rule-1) (psi-get-dual-rules (car (groundable-content-1))))
;            (member (rule-5) (psi-get-dual-rules (car (groundable-content-1)))))
;        #t
;        #f
;    )
;)
; --------------------------------------------------------------
