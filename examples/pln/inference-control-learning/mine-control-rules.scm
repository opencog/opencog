(define (mine-control-rules)
  (icl-logger-fine "Before removing almost false atoms:")
  (icl-logger-fine-atomspace (cog-atomspace))

  ;; Filter history, for now remove all root atoms with TV below
  ;; 0.1. That is because the pattern miner doesn't support values
  ;; yet.
  (remove-almost-false-atoms)

  (icl-logger-fine "After removing almost false atoms:")
  (icl-logger-fine-atomspace (cog-atomspace))

  ;; Apply the pattern miner to that atomspace
  (cog-mine (cog-atomspace) 2 #:maxiter 10 #:initpat (initpat))
)

(define (initpat)
  (LambdaLink
  (VariableList
    (VariableNode "$T")
    (VariableNode "$A")
    (VariableNode "$L")
    (VariableNode "$B")
  )
  (AndLink
    (EvaluationLink
      (PredicateNode "URE:BC:preproof-of")
      (ListLink
        (VariableNode "$A")
        (VariableNode "$T")
      )
    )
    (EvaluationLink
      (PredicateNode "URE:BC:preproof-of")
      (ListLink
        (VariableNode "$B")
        (VariableNode "$T")
      )
    )
    (ExecutionLink
      (SchemaNode "URE:BC:expand-and-BIT")
      (ListLink
        (VariableNode "$A")
        (VariableNode "$L")
        (DontExecLink
          (DefinedSchemaNode "conditional-full-instantiation-implication-scope-meta-rule") ; TODO this shouldn't be hardwired
        )
      )
      (VariableNode "$B")
    )
  )
)
)
