;; Obtained from running
;;
;; (mock-pln-input "Men breathe air")
;;
;; then TVs have been edited at hand so that abduction and deduction
;; work as they should.

(define true-stv (stv 1 1))
(define man-stv (stv 0.01 0.1))

(ReferenceLink
  (PredicateNode "breathe@33c11265-4d02-4c3b-86be-515ad87fba5c" (stv 0.36363634 0.0049751243))
  (WordInstanceNode "breathe@bf2b486b-3dd5-4b50-92e4-20b54d33d226")
)

(InheritanceLink
   (InterpretationNode "sentence@7538a7fe-68a4-44c8-a8e0-554337d90a51_parse_0_interpretation_$X" (stv 0.5 0.0012484394))
   (DefinedLinguisticConceptNode "DeclarativeSpeechAct" (stv 0.09523809 0.0024937654))
)
(EvaluationLink
   (PredicateNode "breathe@33c11265-4d02-4c3b-86be-515ad87fba5c" (stv 0.36363634 0.0049751243))
   (ListLink
      (ConceptNode "men@f8717726-5dde-475f-979c-222790e9fe06" man-stv)
   )
)
(InheritanceLink true-stv
   (ConceptNode "air@0e01ce55-cdd4-4d11-b6f1-9b7bf72c12c3" (stv 0.083333328 0.0024937654))
   (ConceptNode "air" (stv 0.045454547 0.0012484394))
)
(InheritanceLink
   (PredicateNode "breathe@33c11265-4d02-4c3b-86be-515ad87fba5c" (stv 0.36363634 0.0049751243))
   (DefinedLinguisticConceptNode "present" (stv 0.090909086 0.0024937654))
)
(EvaluationLink true-stv
   (PredicateNode "breathe@33c11265-4d02-4c3b-86be-515ad87fba5c" (stv 0.36363634 0.0049751243))
   (ListLink
      (ConceptNode "men@f8717726-5dde-475f-979c-222790e9fe06" man-stv)
      (ConceptNode "air@0e01ce55-cdd4-4d11-b6f1-9b7bf72c12c3" (stv 0.083333328 0.0024937654))
   )
)
(InheritanceLink true-stv
   (ConceptNode "men@f8717726-5dde-475f-979c-222790e9fe06" man-stv)
   (ConceptNode "man" man-stv)
)
(ImplicationLink true-stv
   (PredicateNode "breathe@33c11265-4d02-4c3b-86be-515ad87fba5c" (stv 0.36363634 0.0049751243))
   (PredicateNode "breathe" (stv 0.083333336 0.0012484394))
)
