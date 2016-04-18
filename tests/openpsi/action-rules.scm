; Copyright (C) 2016 OpenCog Foundation

; Define an OpenPsi-Demand called Energy

;; Specify what kind of default behavior the demand should have, using one of
;; the 3 options provided. You can define your own behavior.
;(define energy-default-action  (psi-action-minimize 5))
;
;; Define the Energy demand
;(define energy (psi-demand "Energy" .71 energy-default-action))
;
;; Add action for increasing the energy-demand.
;(psi-action-rule-maximize energy 10 .6)
;(psi-action-rule-maximize energy 5 .3)
;
;; Add action for decreasing the energy-demand.
;(psi-action-rule-minimize energy 6 .6)
;(psi-action-rule-minimize energy 5 .3)
;
;; --------------------------------------------------------------
;; For test_psi_get_action_rules
;(define (test-def) (psi-get-action-rules-typed energy "Default"))
;(define (test-def-result)
;    (SetLink (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default"))
;)
;
;(define (test-inc) (psi-get-action-rules-typed energy "Increase"))
;(define (test-inc-result)
;    (SetLink
;      (DefinedSchemaNode "OpenPsi: Energy-action-rule-maximize-10")
;      (DefinedSchemaNode "OpenPsi: Energy-action-rule-maximize-5")
;    )
;)
;
;(define (test-dec) (psi-get-action-rules-typed energy "Decrease"))
;(define (test-dec-result)
;    (SetLink
;      (DefinedSchemaNode "OpenPsi: Energy-action-rule-minimize-6")
;      (DefinedSchemaNode "OpenPsi: Energy-action-rule-minimize-5")
;    )
;)
;
;; --------------------------------------------------------------
;; For test_psi_get_demands
;(define (test-select-all)
;    ; Using psi-get-demands b/c it is just a wrapper.
;    (psi-get-demands-all)
;)
;
;(define (test-select-all-result)
;    (SetLink
;      (ConceptNode "OpenPsi: Energy" (stv 0.710000 1.000000))
;    )
;)

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
