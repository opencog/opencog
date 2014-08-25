;; Converting a multi-root graph to a single root graph, it's easier to run Breadth-first search on such graph.

(define connectRootsToParseNodes
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$top relationship")
                (ListLink
                    (VariableTypeNode "DefinedLinguisticRelationshipNode")
                    (VariableTypeNode "PrepositionalRelationshipNode")
                )
            )
            (TypedVariableLink
                (VariableNode "$other relationship")
                (ListLink
                    (VariableTypeNode "DefinedLinguisticRelationshipNode")
                    (VariableTypeNode "PrepositionalRelationshipNode")
                )
            )
            (TypedVariableLink
                (VariableNode "$parse node")
                (VariableTypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$parent")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$root")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$child")
                (VariableTypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
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
                (NotLink
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
)
