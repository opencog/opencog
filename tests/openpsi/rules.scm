; Copyright (C) 2016 OpenCog Foundation

; --------------------------------------------------------------
; If you want to run this in guile wihout installing,
; 1. run cmake in the build directory
; 2. run  (add-to-load-path "/absolute/path/to/build/opencog/scm")
; 3. (use-modules (opencog) (opencog openpsi))

; --------------------------------------------------------------
(define context-1
    (list
       (ListLink
            (VariableNode "x")
            (VariableNode "y")
            (ConceptNode "Required constant for DualLink")
            (VariableNode "z"))
        (InheritanceLink
            (VariableNode "x")
            (VariableNode "z"))
        (EqualLink
            (VariableNode "x")
            (VariableNode "y"))
        ))

(define action-1
    (ExecutionOutputLink
        (GroundedSchemaNode "scm: act-1")
        (ListLink)))

(define (act-1)
    (ConceptNode "act-1")
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
            (ConceptNode "Required constant for DualLink")
            (PredicateNode "z"))
        (InheritanceLink
            (NumberNode 1)
            (PredicateNode "z")))
)

; --------------------------------------------------------------
(define context-2
    (list ; They are in a list so as to simplify removal.
       (ListLink
            (VariableNode "x")
            (ConceptNode "Required constant for DualLink")
            (VariableNode "z"))
        (InheritanceLink
            (VariableNode "x")
            (VariableNode "z"))
        (NotLink (EqualLink
            (VariableNode "x")
            (VariableNode "z")))
        ))

(define action-2
    (ExecutionOutputLink
        (GroundedSchemaNode "scm: act-2")
        (ListLink)))

(define (act-2)
    (ConceptNode "act-2")
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
            (ConceptNode "Required constant for DualLink")
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
; Helper functions for `OpenPsiUTest::test_psi_step`
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
    ; Load groundable contents for satisfying rule-2 only
    (groundable-content-1)
    ; Make one step
    (psi-step)
)

(define (test_psi_step_2_1) (cog-node? (cog-node 'ConceptNode "act-1")))

(define (setup_test_psi_step_3)
    ; Clean the atomspace so that only rule-2 and rule-3 are selectable
    (map cog-delete (groundable-content-1))
    (cog-delete (act-1))
    ; Change the strength of demand-1 to its initial value
    (cog-set-tv! demand-1 (stv .87 1))
    ; Load groundable contents for satisfying rule-2 or rule 3
    (groundable-content-2)
    ; Make one step
    (psi-step)
)

(define (test_psi_step_3_1) (cog-node? (cog-node 'ConceptNode "act-2")))
