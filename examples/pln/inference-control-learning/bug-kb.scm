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
