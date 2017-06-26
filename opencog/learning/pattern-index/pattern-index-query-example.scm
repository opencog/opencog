(use-modules (opencog))
(use-modules (opencog learning pattern-index))

(let (
    (index-key  (pi-create-index (ConceptNode "../opencog/learning/pattern-index/toy-example-query.scm")))
    (query1 
        (AndLink 
            (SimilarityLink (VariableNode "X") (VariableNode "Y")) 
            (SimilarityLink (VariableNode "Y") (VariableNode "Z"))
        )
    )
    (query2 
        (AndLink 
            (SimilarityLink (VariableNode "X") (VariableNode "Y")) 
            (NotLink 
                (AndLink 
                    (InheritanceLink (VariableNode "X") (VariableNode "Z")) 
                    (InheritanceLink (VariableNode "Y") (VariableNode "Z"))
                )
            )
        )
    )
) (begin
    (display "query1: ")
    (display query1)
    (display "Result: ")
    (display (pi-query index-key query1))
    (display "query2: ")
    (display query2)
    (display "Result: ")
    (display (pi-query index-key query2))
))
