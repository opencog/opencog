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

(ure-add-rule initial-rbs "initial-rule" initial-rule 1)
