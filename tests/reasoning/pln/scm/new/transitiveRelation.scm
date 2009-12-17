; Adapted from bigdemo.scm by JaredW
(ForAllLink (stv 1 0.99900001) (ListLink (stv 1 0) (VariableNode "X011" (stv 1 0))
    (VariableNode "X010" (stv 1 0))
    (VariableNode "X009" (stv 1 0))
    (VariableNode "R001" (stv 1 0)))
    (ImplicationLink (stv 1 0) (AndLink (stv 1 0) (InheritanceLink (stv 1 0) (VariableNode "R001" (stv 1 0))
    (ConceptNode "transitiveRelation" (stv 1 0)))
    (AndLink (stv 1 0) (EvaluationLink (stv 1 0) (VariableNode "R001" (stv 1 0))
    (ListLink (stv 1 0) (VariableNode "X009" (stv 1 0))
       (VariableNode "X010" (stv 1 0))))
       (EvaluationLink (stv 1 0) (VariableNode "R001" (stv 1 0))
          (ListLink (stv 1 0) (VariableNode "X010" (stv 1 0))
             (VariableNode "X011" (stv 1 0))))))
       (EvaluationLink (stv 1 0) (VariableNode "R001" (stv 1 0))
          (ListLink (stv 1 0) (VariableNode "X009" (stv 1 0))
             (VariableNode "X011" (stv 1 0))))))

(InheritanceLink (stv 0.89999998 0.89999998) (PredicateNode "friendOf" (stv 1 0))
    (ConceptNode "transitiveRelation" (stv 1 0)))

(EvaluationLink (stv 0.80000001 0.2) (PredicateNode "friendOf" (stv 1 0))
    (ListLink (stv 1 0) (ConceptNode "Bill" (stv 1 0))
       (ConceptNode "Ted" (stv 1 0))))
(EvaluationLink (stv 0.80000001 0.2) (PredicateNode "friendOf" (stv 1 0))
    (ListLink (stv 1 0) (ConceptNode "Ted" (stv 1 0))
       (ConceptNode "John" (stv 1 0))))

(define t (EvaluationLink (PredicateNode "friendOf" (stv 1 0))
    (ListLink (stv 1 0) (ConceptNode "Bill" (stv 1 0))
       (ConceptNode "John" (stv 1 0)))))



; (pln-bc t 2400)
