; Copyright (C) 2015-2016 OpenCog Foundation

(use-modules (opencog) (opencog rule-engine))

; Create a rulebase
(define initial-rbs (ConceptNode "initial-rbs"))
(ure-define-rbs initial-rbs 1)

; Add a rule to initial-rbs
; NOTE: The rule should be updated as the primitive atom-type(or term) behaviors
; (virtual-link, evaluatable, function-link, ...) are updated/extended or,
; new behaviors added.
(define rule-1
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "x")
                (TypeNode "NumberNode"))
            (TypedVariableLink
                (VariableNode "y")
                (TypeNode "NumberNode"))
            (TypedVariableLink
                (VariableNode "z")
                (TypeNode "PredicateNode")))
        (AndLink
            (ListLink
                (VariableNode "x")
                (VariableNode "y")
                (VariableNode "z"))
            (InheritanceLink
                (VariableNode "x")
                (VariableNode "z"))
            ; Virtual-link
            (EqualLink
                (VariableNode "x")
                (VariableNode "y"))
            ; Evaluatable term
            (EvaluationLink
                (GroundedPredicateNode "scm: cog-tv")
                (ListLink
                    (VariableNode "z")))
            ; Function-link
            (TimesLink
                (VariableNode "x")
                (VariableNode "y")))
        (ListLink
             (SetLink
                 (VariableNode "x")
                 (VariableNode "y")
                 (VariableNode "z"))))
)

(define rule1-alias (ure-define-add-rule initial-rbs "rule-1" rule-1 1))

; FIXME: Add random inheritance for checking the filtering of multiple
; inheritance from different atoms. This atom should be added first, so as to
; break `ActionUTest::test_constuctor` test.
; NOTE: Look into ForeachChaseLink.h for a fix, without needing an atomspace
; similar to cog-chase-link.
; (InheritanceLink rule-alias (ConceptNode "breaking-test-node"))

(InheritanceLink rule1-alias (ConceptNode "opencog: action"))


; Helper function for `ActionUTest::test_is_derived_state_satisfiable`
(define (add-content)
    (ListLink
        (NumberNode 1)
        (NumberNode 2)
        (PredicateNode "z"))
    (InheritanceLink
        (NumberNode 1)
        (PredicateNode "z"))
)

; This rule has the same pattern as `rule-1` so, on context based
; selection it is as likely to be selected.
(define rule-2
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "x2")
                (TypeNode "NumberNode"))
            (TypedVariableLink
                (VariableNode "y2")
                (TypeNode "NumberNode"))
            (TypedVariableLink
                (VariableNode "z2")
                (TypeNode "PredicateNode")))
        (AndLink
            (ListLink
                (VariableNode "x2")
                (VariableNode "y2")
                (VariableNode "z2"))
            (InheritanceLink
                (VariableNode "x2")
                (VariableNode "z2"))
            ; Virtual-link
            (EqualLink
                (VariableNode "x2")
                (VariableNode "y2"))
            ; Evaluatable term
            (EvaluationLink
                (GroundedPredicateNode "scm: cog-tv")
                (ListLink
                    (VariableNode "z2")))
            ; Function-link
            (TimesLink
                (VariableNode "x2")
                (VariableNode "y2")))
        (ListLink
             (SetLink
                 (VariableNode "x2")
                 (VariableNode "y2")
                 (VariableNode "z2"))))
)

(define rule2-alias (ure-define-add-rule initial-rbs "rule-2" rule-2 1))

; Helper function for `ActionSelectorUTest`
(define (convert-to-action-rule alias)
"
  Takes a rule alias and makes it an opencog action.
"
    (InheritanceLink alias (ConceptNode "opencog: action"))
)
