(define pln-rule-equivalence-hack
  (BindLink
     (ImplicationLink
        (EvaluationLink
           (PredicateNode "take")
           (ListLink
              (QuoteLink (VariableNode "X"))
              (PredicateNode "treatment-1")
           )
        )
        (EvaluationLink
           (PredicateNode "take")
           (ListLink
              (QuoteLink (VariableNode "X"))
              (PredicateNode "compound-A")
           )
        )
     )
     (ImplicationLink (stv 1 1)
         (PredicateNode "take-treatment-1")
         (PredicateNode "take-compound-A")
     )
  )
)

(define pln-rule-equivalence-hack-name (Node "pln-rule-equivalence-hack"))
(DefineLink pln-rule-equivalence-hack-name pln-rule-equivalence-hack)

(define pln-rule-eliminate-neutral-element-hack
  (BindLink
     (ImplicationLink
        (AndLink
           (EvaluationLink
              (PredicateNode "take")
              (ListLink
                 (QuoteLink (VariableNode "X"))
                 (PredicateNode "treatment-1")
              )
           )
           (EvaluationLink
              (PredicateNode "contain")
              (ListLink
                 (PredicateNode "treatment-1")
                 (PredicateNode "compound-A")
              )
           )
        )
        (EvaluationLink
           (PredicateNode "take")
           (ListLink
              (QuoteLink (VariableNode "X"))
              (PredicateNode "compound-A")
           )
        )
     )
     (ImplicationLink (stv 1 1)
        (AndLink
           (EvaluationLink
              (PredicateNode "take")
              (ListLink
                 (VariableNode "X")
                 (PredicateNode "treatment-1")
              )
           )
        )
        (EvaluationLink
           (PredicateNode "take")
           (ListLink
              (VariableNode "X")
              (PredicateNode "compound-A")
           )
        )
     )
  )
)

(define pln-rule-eliminate-neutral-element-hack-name
  (Node "pln-rule-eliminate-neutral-element-hack"))
(DefineLink
  pln-rule-eliminate-neutral-element-hack-name
  pln-rule-eliminate-neutral-element-hack)

(define pln-rule-eliminate-dangling-junctor-hack
  (BindLink
     (ImplicationLink
        (AndLink
           (EvaluationLink
              (PredicateNode "take")
              (ListLink
                 (QuoteLink (VariableNode "X"))
                 (PredicateNode "treatment-1")
              )
           )
        )
        (EvaluationLink
           (PredicateNode "take")
           (ListLink
              (QuoteLink (VariableNode "X"))
              (PredicateNode "compound-A")
           )
        )
     )
     (ImplicationLink (stv 1 1)
        (EvaluationLink
           (PredicateNode "take")
           (ListLink
              (VariableNode "X")
              (PredicateNode "treatment-1")
           )
        )
        (EvaluationLink
           (PredicateNode "take")
           (ListLink
              (VariableNode "X")
              (PredicateNode "compound-A")
           )
        )
     )
  )
)

(define pln-rule-eliminate-dangling-junctor-hack-name
  (Node "pln-rule-eliminate-dangling-junctor-hack"))
(DefineLink
  pln-rule-eliminate-dangling-junctor-hack-name
  pln-rule-eliminate-dangling-junctor-hack)

(define pln-rule-and-hack
(BindLink
   (AndLink
      (EvaluationLink
         (PredicateNode "take")
         (ListLink
            (QuoteLink (VariableNode "X"))
            (VariableNode "Y")
         )
      )
      (EvaluationLink
         (PredicateNode "contain")
         (ListLink
            (VariableNode "Y")
            (VariableNode "Z")
         )
      )
   )
   (AndLink (stv 1 1)
      (EvaluationLink
         (PredicateNode "take")
         (ListLink
            (VariableNode "X")
            (VariableNode "Y")
         )
      )
      (EvaluationLink
         (PredicateNode "contain")
         (ListLink
            (VariableNode "Y")
            (VariableNode "Z")
         )
      )
   )
)
)

(define pln-rule-and-hack-name
  (Node "pln-rule-and-hack"))
(DefineLink
  pln-rule-and-hack-name
  pln-rule-and-hack)

(define pln-rule-for-all-hack
  (BindLink
     (ForAllLink
        (ListLink
           (QuoteLink (VariableNode "X"))
           (QuoteLink (VariableNode "Y"))
           (QuoteLink (VariableNode "Z"))
        )
        (ImplicationLink
           (AndLink
              (EvaluationLink
                 (PredicateNode "take")
                 (ListLink
                    (QuoteLink (VariableNode "X"))
                    (QuoteLink (VariableNode "Y"))
                 )
              )
              (EvaluationLink
                 (PredicateNode "contain")
                 (ListLink
                    (QuoteLink (VariableNode "Y"))
                    (QuoteLink (VariableNode "Z"))
                 )
              )
           )
           (EvaluationLink
              (PredicateNode "take")
              (ListLink
                 (QuoteLink (VariableNode "X"))
                 (QuoteLink (VariableNode "Z"))
              )
           )
        )
     )
     (ImplicationLink (stv 1 1)
        (AndLink
           (EvaluationLink
              (PredicateNode "take")
              (ListLink
                 (VariableNode "X")
                 (PredicateNode "treatment-1")
              )
           )
           (EvaluationLink
              (PredicateNode "contain")
              (ListLink
                 (PredicateNode "treatment-1")
                 (PredicateNode "compound-A")
              )
           )
        )
        (EvaluationLink
           (PredicateNode "take")
           (ListLink
              (VariableNode "X")
              (PredicateNode "compound-A")
           )
        )
     )
  )
)

(define pln-rule-for-all-hack-name
  (Node "pln-rule-for-all-hack"))
(DefineLink
  pln-rule-for-all-hack-name
  pln-rule-for-all-hack)

(define pln-rule-average-hack
  (BindLink
     (AverageLink
        (QuoteLink (VariableNode "X"))
        (ImplicationLink
           (MemberLink
              (QuoteLink (VariableNode "X"))
              (ConceptNode "injury-recovery-speed-predicates")
           )
           (ImplicationLink
              (PredicateNode "is-well-hydrated")
              (QuoteLink (VariableNode "X"))
           )
        )
     )
     (ImplicationLink (stv 0.7 0.6)
        (MemberLink
           (PredicateNode "recovery-speed-of-injury-alpha")
           (ConceptNode "injury-recovery-speed-predicates")
        )
        (ImplicationLink
           (PredicateNode "is-well-hydrated")
           (PredicateNode "recovery-speed-of-injury-alpha")
        )
     )
  )
)

(define pln-rule-average-hack-name
  (Node "pln-rule-average-hack"))
(DefineLink
  pln-rule-average-hack-name
  pln-rule-average-hack)
