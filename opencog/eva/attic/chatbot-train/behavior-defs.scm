;-----------------------------------------------------------------
; Define behavior actions

; happy
(DefineLink
    (DefinedPredicateNode "be happy")
    (EvaluationLink
        (DefinedPredicateNode "Do show expression")
        (ListLink
          (ConceptNode "imperative")
          (ConceptNode "happy"))))

(DefineLink
    (DefinedPredicateNode "happy")
    (EvaluationLink
        (DefinedPredicateNode "Do show expression")
        (ListLink
          (ConceptNode "imperative")
          (ConceptNode "happy"))))

; yawn
(DefineLink
    (DefinedPredicateNode "yawn")
    (EvaluationLink
        (DefinedPredicateNode "Do show gesture")
        (ListLink
            (ConceptNode "imperative")
            (ConceptNode "yawn-1"))))

