;; Obtained from running
;;
;; (mock-pln-input "John is a man")
;;
;; then TVs have been edited at hand so that abduction and deduction
;; work as they should.

(define true-stv (stv 1 1))
(define man-stv (stv 0.01 0.1))
(define john-stv (stv 0.001 0.1))

(InheritanceLink true-stv
   (ConceptNode "man@5d693a26-661d-4c15-844c-c1c7c34281c4" man-stv)
   (ConceptNode "man" man-stv)
)
(InheritanceLink
   (SpecificEntityNode "John@326e86cc-c68a-4030-ab89-444a88a249ce" john-stv)
   (DefinedLinguisticConceptNode "male" (stv 0.055555556 0.0012484394))
)
(InheritanceLink
   (InterpretationNode "sentence@1a59ced0-01c9-4511-972d-70cf0bcb2ee9_parse_0_interpretation_$X" (stv 1 0.0012484394))
   (DefinedLinguisticConceptNode "DeclarativeSpeechAct" (stv 0.09523809 0.0024937654))
)
(EvaluationLink
   (DefinedLinguisticPredicateNode "definite" (stv 1 0.0012484394))
   (ListLink
      (ConceptNode "John@326e86cc-c68a-4030-ab89-444a88a249ce" john-stv)
   )
)
(InheritanceLink true-stv
   (ConceptNode "John@326e86cc-c68a-4030-ab89-444a88a249ce" john-stv)
   (ConceptNode "man@5d693a26-661d-4c15-844c-c1c7c34281c4" man-stv)
)
(EvaluationLink
   (PredicateNode "is@6586f2ba-d712-4cb0-a22b-2abe5dcb45ec" (stv 0.66666663 0.0049751243))
   (ListLink
      (ConceptNode "John@326e86cc-c68a-4030-ab89-444a88a249ce" john-stv)
   )
)
(EvaluationLink
   (PredicateNode "is@6586f2ba-d712-4cb0-a22b-2abe5dcb45ec" (stv 0.66666663 0.0049751243))
   (ListLink
      (ConceptNode "John@326e86cc-c68a-4030-ab89-444a88a249ce" john-stv)
      (ConceptNode "man@5d693a26-661d-4c15-844c-c1c7c34281c4" man-stv)
   )
)
(InheritanceLink
   (PredicateNode "is@6586f2ba-d712-4cb0-a22b-2abe5dcb45ec" (stv 0.66666663 0.0049751243))
   (DefinedLinguisticConceptNode "present" (stv 0.090909086 0.0024937654))
)
(InheritanceLink true-stv
   (ConceptNode "John@326e86cc-c68a-4030-ab89-444a88a249ce" john-stv)
   (ConceptNode "John" john-stv)
)
(InheritanceLink
   (SpecificEntityNode "John@326e86cc-c68a-4030-ab89-444a88a249ce" john-stv)
   (ConceptNode "John" john-stv)
)
(ImplicationLink
   (PredicateNode "is@6586f2ba-d712-4cb0-a22b-2abe5dcb45ec" (stv 0.66666663 0.0049751243))
   (PredicateNode "be" (stv 0.14285715 0.0012484394))
)
