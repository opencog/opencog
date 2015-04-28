
;; Test dataset, in scheme ...

  (AtTimeLink
     (TimeNode "630057840")
     (ConceptNode "07620189")
  )

  (AndLink
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "07620189")
        (VariableNode "X")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "07620189")
        (NumberNode "1")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "12886371")
        (VariableNode "X")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "12886371")
        (NumberNode "2")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "12878615")
        (VariableNode "X")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "12878615")
        (NumberNode "3")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "00069143")
        (VariableNode "X")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "00069143")
        (NumberNode "4")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "13792261")
        (VariableNode "Z")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "13792261")
        (NumberNode "5")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "00959470")
        (VariableNode "Z")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "00959470")
        (NumberNode "6")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "00131306")
        (VariableNode "Z")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "00131306")
        (NumberNode "7")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "00080765")
        (VariableNode "Z")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "00080765")
        (NumberNode "8")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "00049502")
        (VariableNode "Z")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "00049502")
        (NumberNode "9")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "01347204")
        (VariableNode "X")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "01347204")
        (NumberNode "10")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "01129413")
        (VariableNode "X")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "01129413")
        (NumberNode "11")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "00065349")
        (VariableNode "Y")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "00065349")
        (NumberNode "12")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptVariablePredicate")
      (ListLink
        (ConceptNode "08042383")
        (VariableNode "Y")
      )
    )
    (EvaluationLink
      (PredicateNode "ConceptWordPredicate")
      (ListLink
        (ConceptNode "08042383")
        (NumberNode "13")
      )
    )
  )

  ;; <!-- atoms to test substitutableTo when comparing unordered links -->
  (SetLink
      (PredicateNode "ConceptVariablePredicate")
      (ConceptNode "08042383")
      (PredicateNode "ConceptWordPredicate")
  )

  ;; <!-- Inserts another SetLink with the same elements in the outgoingset, 
  ;;    but in a different order. This link must be considered identical 
  ;;     by OpenCog since SetLink is an unordered link. -->
  (SetLink
      (PredicateNode "ConceptWordPredicate")
      (PredicateNode "ConceptVariablePredicate")
      (ConceptNode "08042383")
  )

