;; Converting a multi-root graph to a single root graph, it's easier to run Breadth-first search on such graph.

(define connectRootsToParseNodes
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$top relationship")
                (TypeChoice
                    (TypeNode "DefinedLinguisticRelationshipNode")
                    (TypeNode "PrepositionalRelationshipNode")
                )
            )
            (TypedVariableLink
                (VariableNode "$other relationship")
                (TypeChoice
                    (TypeNode "DefinedLinguisticRelationshipNode")
                    (TypeNode "PrepositionalRelationshipNode")
                )
            )
            (TypedVariableLink
                (VariableNode "$parse node")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$parent")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$root")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$child")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
           (WordInstanceLink
                (VariableNode "$root")
                (VariableNode "$parse node")
           )
           (EvaluationLink
                (VariableNode "$top relationship")
                (ListLink
                    (VariableNode "$root")
                    (VariableNode "$child")
                )
            )
            (AbsentLink
                (EvaluationLink
                    (VariableNode "$other relationship")
                    (ListLink
                        (VariableNode "$parent")
                        (VariableNode "$root")
                    )
                )
            )
        )
        (EvaluationLink
            (DefinedLinguisticRelationshipNode "__temp__")
            (ListLink
                (VariableNode "$parse node")
                (VariableNode "$root")
            )
        )
    )
)
