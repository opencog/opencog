; Copyright (C) 2016 OpenCog Foundation

; --------------------------------------------------------------
; If you want to run this in guile wihout installing,
; 1. run cmake in the build directory
; 2. run  (add-to-load-path "/absolute/path/to/build/opencog/scm")
; 3. (use-modules (opencog) (opencog openpsi))
; --------------------------------------------------------------
; Helper function for `OpenPsiUTest::test_psi_satisfiable?`
(define context-1
    (list
       (ListLink
            (VariableNode "x")
            (VariableNode "y")
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
    (ListLink
        (NumberNode 1)
        (NumberNode 1)
        (PredicateNode "z"))
    (InheritanceLink
        (NumberNode 1)
        (PredicateNode "z"))
)

; --------------------------------------------------------------
(define context-2
    (list
       (ListLink
            (VariableNode "x")
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
    (ListLink
        (NumberNode 1)
        (NumberNode 2))
    (InheritanceLink
        (NumberNode 1)
        (NumberNode 2))
)
