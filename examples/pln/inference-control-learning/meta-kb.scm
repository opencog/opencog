;; Meta knowledge-base, what we need to know to infer inference
;; control rules. Such as what are preproofs.
;;
;; 1. Any and-BIT has a very low probability, 0.0001%, of being the
;;    preproof of any target, and we have a very low confidence,
;;    0.0001, about this probability. These numbers are made up. This
;;    should be replaced by a set of more informative laws about the
;;    space of proofs, so that we can for instance infer that small
;;    and-BITs are more likely to be preproofs than larger and-BITs,
;;    etc.
;;
;; ImplicationScope <0.0001 0.001>
;;   VariableList
;;     TypedVariable
;;       Variable "$A"
;;       Type "BindLink"
;;     Variable "$T"
;;   And
;;     Evaluation
;;       Predicate "ICL:and-BIT"
;;       Variable "$A"
;;     Evaluation
;;       Predicate "ICL:target"
;;       Variable "$T"
;;   Evaluation
;;     Predicate "ICL:preproof"
;;     List
;;       Variable "$A"
;;       Variable "$T"
;;
;; 2. If and-BIT A is a proof of target T, then it is a preproof of
;;    target T as well.
;;
;; Implication <1 1>
;;   Predicate "URE:BC:proof"
;;   Predicate "ICL:preproof"
;;
;; 3. If and-BIT A expands into and-BIT B that is a preproof of target T,
;;    then A is a preproof of T as well.
;;
;; ImplicationScope <1 1>
;;   VariableList
;;     TypedVariable
;;       Variable "$A"
;;       Type "BindLink"
;;     TypedVariable
;;       Variable "$B"
;;       Type "BindLink"
;;     TypedVariable
;;       Variable "$R"
;;       Type "BindLink"
;;     Variable "$L"
;;     Variable "$T"
;;   And
;;     Execution
;;       Schema "URE:BC:expand-and-BIT"
;;       List
;;         Variable "$A"
;;         Variable "$L"
;;         Variable "$R"
;;       Variable "$B"
;;     Evaluation
;;       Predicate "ICL:preproof"
;;       List
;;         Variable "$B"
;;         Variable "$T"
;;   Evaluation
;;     Predicate "ICL::preproof"
;;     List
;;       Variable "$A"
;;       Variable "$T"
;;
;; We could add more knowledge such that initial and-BITs like
;;
;; Bind
;;   T
;;   T
;;
;; are preproof if there exists a proof of T but it doesn't appear
;; immediately obviously useful so we let that aside.

;; 1. And-BIT prior
(ImplicationScope (stv 0.0001 0.001)
  (VariableList
    (TypedVariable
      (Variable "$A")
      (Type "BindLink"))
    (Variable "$T"))
  (And
    (Evaluation
      (Predicate "ICL:and-BIT")
      (Variable "$A"))
    (Evaluation
      (Predicate "ICL:target")
      (Variable "$T")))
  (Evaluation
    (Predicate "ICL:preproof")
    (List
      (Variable "$A")
      (Variable "$T"))))

;; 2. If and-BIT A is a proof of target T, then it is a preproof of T.
(Implication (stv 1 1)
  (Predicate "URE:BC:proof")
  (Predicate "ICL:preproof"))

;; 3. If and-BIT A expands into and-BIT B that is a preproof of target T,
;;    then A is a preproof of T.
(ImplicationScope (stv 1 1)
  (VariableList
    (TypedVariable
      (Variable "$A")
      (Type "BindLink"))
    (TypedVariable
      (Variable "$B")
      (Type "BindLink"))
    (TypedVariable
      (Variable "$R")
      (Type "BindLink"))
    (Variable "$L")
    (Variable "$T"))
  (And
    (Execution
      (Schema "URE:BC:expand-and-BIT")
      (List
        (Variable "$A")
        (Variable "$L")
        (Variable "$R"))
      (Variable "$B"))
    (Evaluation
      (Predicate "ICL:preproof")
      (List
        (Variable "$B")
        (Variable "$T"))))
  (Evaluation
    (Predicate "ICL::preproof")
    (List
      (Variable "$A")
      (Variable "$T"))))
