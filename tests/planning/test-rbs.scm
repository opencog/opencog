(use-modules (opencog) (opencog rule-engine))

; Create a rulebase
(define initial-rbs (ConceptNode "initial-rbs"))
(ure-define-rbs initial-rbs 1)

; Add a rule to initial-rbs
; NOTE: The rule should be updated as the primitive atom-type(or term) behaviors
; (virtual-link, evaluatable, function-link, ...) are updated/extended or,
; new behaviors added.
(define initial-rule
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

(define rule-alias (ure-add-rule initial-rbs "initial-rule" initial-rule 1))

; FIXME: Add random inheritance for checking the filtering of multiple
; inheritance from different atoms. This atom should be added first, so as to
; break `test_constuctor` test.
; NOTE: Look into ForeachChaseLink.h for a fix, without needing an atomspace
; similar to cog-chase-link.
; (InheritanceLink rule-alias (ConceptNode "breaking-test-node"))

(InheritanceLink rule-alias (ConceptNode "opencog: action"))
