(ConceptNode "abc")
#!
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
            ; virtual-link
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

(define rule2
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "x")
                (TypeNode "NumberNode"))
            (TypedVariableLink
                (VariableNode "y")
                (TypeNode "NumberNode")))

        (AndLink
            ; virtual-link
            (ListLink
                (VariableNode "x")
                (VariableNode "y"))
            ; Evaluatable
            (ListLink
                (VariableNode "z")
                (VariableNode "what")))
        (ListLink
             (SetLink
                 (VariableNode "x")
                 (VariableNode "y"))))
 )

(define rule3
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "x")
                (TypeNode "NumberNode"))
            (TypedVariableLink
                (VariableNode "y")
                (TypeNode "NumberNode")))

        (AndLink
            ; virtual-link

            ; Evaluatable
            (ListLink
                (VariableNode "z")
                (VariableNode "what")))
        (ListLink
             (SetLink
                 (VariableNode "z")
                 (VariableNode "what"))))
)

(NumberNode "1")
(NumberNode "2")

; Matching atom
(ListLink
    (ConceptNode "x")
    (ConceptNode "x")
    (PredicateNode "y"))

; Random atom
(ListLink
    (ConceptNode "x")
    (ConceptNode "y")
    (ConceptNode "x"))

!#
