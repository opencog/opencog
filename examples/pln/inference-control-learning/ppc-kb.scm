;; Knowledge-base about preproof and anything else we need to
;; post-process the trace atomspace before handing it to the process
;; in charge of producing inference control rules. ppc stands for
;; post-process corpus.
;;
;; 1. Any and-BIT has a very low probability, 0.0001%, of being the
;;    preproof of any target, and we have a very low confidence, 0.01,
;;    about this probability. These numbers are made up. This should
;;    be replaced by a set of more informative laws about the space of
;;    proofs, so that we can for instance infer that small and-BITs
;;    are more likely to be preproofs than larger and-BITs, etc.
;;
;; ImplicationScope <0.0001 0.01>
;;   VariableList
;;     TypedVariable
;;       Variable "$A"
;;       Type "BindLink"
;;     Variable "$T"
;;   And
;;     Evaluation
;;       Predicate "URE:BC:and-BIT"
;;       Variable "$A"
;;     Evaluation
;;       Predicate "URE:BC:target"
;;       Variable "$T"
;;   Evaluation
;;     Predicate "URE:BC:preproof"
;;     List
;;       Variable "$A"
;;       Variable "$T"
;;
;;    This prior is important because otherwise instances of and-BIT
;;    not being found as preproof will have a null confidence, that is
;;    because we cannot know with certainty that an and-BIT is *not* a
;;    preproof as it may require to explore the entire univers of
;;    proofs to rule it out. But a null confidence would count as
;;    nothing (according to an extension of the formula in Section
;;    2.4.1.1 of the PLN book to consider uncertainty) which would
;;    make it hard to compare the performances of two predictors since
;;    if they have only positive examples, then the one with the
;;    higher number of examples would win, even if it less accurate,
;;    which isn't right. Indeed the reason is because not finding a
;;    proof does actually bring a bit of information, since there are
;;    so many more non-proofs that proofs, a random and-BIT is likely
;;    not a preproof. That is what this prior captures.
;;
;; 2. If and-BIT A is a proof of target T, then it is a preproof of
;;    target T as well.
;;
;; Implication <1 1>
;;   Predicate "URE:BC:proof"
;;   Predicate "URE:BC:preproof"
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
;;       Predicate "URE:BC:preproof"
;;       List
;;         Variable "$B"
;;         Variable "$T"
;;   Evaluation
;;     Predicate "URE:BC:preproof"
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
;;
;; Technical remark: and-BITs, rules are wrapped by DontExecLinks to
;; garanty that they won't be executed while reasoning on them.

;; 1. And-BIT prior
(ImplicationScope (stv 0.0001 0.001)
  (VariableList
    (TypedVariable
      (Variable "$A")
      (Type "DontExecLink"))
    (Variable "$T"))
  (And
    (Evaluation
      (Predicate "URE:BC:and-BIT")
      (Variable "$A"))
    (Evaluation
      (Predicate "URE:BC:target")
      (Variable "$T")))
  (Evaluation
    (Predicate "URE:BC:preproof")
    (List
      (Variable "$A")
      (Variable "$T"))))

;; 2. If and-BIT A is a proof of target T, then it is a preproof of T.
(Implication (stv 1 1)
  (Predicate "URE:BC:proof")
  (Predicate "URE:BC:preproof"))

;; 3. If and-BIT A expands into and-BIT B that is a preproof of target T,
;;    then A is a preproof of T.
(ImplicationScope (stv 1 1)
  (VariableList
    (TypedVariable
      (Variable "$A")
      (Type "DontExecLink"))
    (TypedVariable
      (Variable "$B")
      (Type "DontExecLink"))
    (TypedVariable
      (Variable "$R")
      (Type "DontExecLink"))
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
      (Predicate "URE:BC:preproof")
      (List
        (Variable "$B")
        (Variable "$T"))))
  (Evaluation
    (Predicate "URE:BC:preproof")
    (List
      (Variable "$A")
      (Variable "$T"))))
