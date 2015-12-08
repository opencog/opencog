(use-modules (opencog) (opencog rule-engine))

; Create a rulebase
(define initial-rbs (ConceptNode "initial-rbs"))
(ure-define-rbs initial-rbs 1)

; Add a rule to initial-rbs
(define initial-rule
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "x")
                (TypeNode "ConceptNode"))
            (TypedVariableLink
                (VariableNode "y")
                (TypeNode "ConceptNode"))
            (TypedVariableLink
                (VariableNode "z")
                (TypeNode "PredicateNode")))
        (AndLink
            (ListLink
                (VariableNode "x")
                (VariableNode "y")
                (VariableNode "z"))
            ; Function-link
            (EqualLink
                (VariableNode "x")
                (VariableNode "y")))
            ; Evaluatable
            #!(EvaluationLink)
             (InheritanceLink
                 )!#
        (ListLink
             (SetLink
                 (VariableNode "x")
                 (VariableNode "y"))))
)

(ure-add-rule initial-rbs "initial-rule" initial-rule 1)
