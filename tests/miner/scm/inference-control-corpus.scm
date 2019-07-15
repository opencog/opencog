(EvaluationLink (stv 1 1)
   (PredicateNode "URE:BC:preproof-of")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "a")
               (ConceptNode "z")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "a")
         (ConceptNode "z")
      )
   )
)
(EvaluationLink (stv 1 1)
   (PredicateNode "URE:BC:preproof-of")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
               (EvaluationLink
                  (GroundedPredicateNode "scm: gt-zero-confidence")
                  (EvaluationLink (stv 1 1)
                     (PredicateNode "alphabetical-order")
                     (ListLink
                        (ConceptNode "a")
                        (ConceptNode "z")
                     )
                  )
               )
            )
            (ExecutionOutputLink
               (GroundedSchemaNode "scm: conditional-full-instantiation-scope-formula")
               (ListLink
                  (InheritanceLink
                     (ConceptNode "a")
                     (ConceptNode "z")
                  )
                  (EvaluationLink (stv 1 1)
                     (PredicateNode "alphabetical-order")
                     (ListLink
                        (ConceptNode "a")
                        (ConceptNode "z")
                     )
                  )
                  (ImplicationScopeLink (stv 1 1)
                     (VariableList
                        (TypedVariableLink
                           (VariableNode "$X")
                           (TypeNode "ConceptNode")
                        )
                        (TypedVariableLink
                           (VariableNode "$Y")
                           (TypeNode "ConceptNode")
                        )
                     )
                     (EvaluationLink
                        (PredicateNode "alphabetical-order")
                        (ListLink
                           (VariableNode "$X")
                           (VariableNode "$Y")
                        )
                     )
                     (InheritanceLink
                        (VariableNode "$X")
                        (VariableNode "$Y")
                     )
                  )
               )
            )
         )
      )
      (InheritanceLink
         (ConceptNode "a")
         (ConceptNode "z")
      )
   )
)
(EvaluationLink (stv 1 1)
   (PredicateNode "URE:BC:preproof-of")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "a")
               (ConceptNode "g")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "a")
         (ConceptNode "g")
      )
   )
)
(EvaluationLink (stv 1 1)
   (PredicateNode "URE:BC:preproof-of")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
               (EvaluationLink
                  (GroundedPredicateNode "scm: gt-zero-confidence")
                  (EvaluationLink (stv 1 1)
                     (PredicateNode "alphabetical-order")
                     (ListLink
                        (ConceptNode "a")
                        (ConceptNode "g")
                     )
                  )
               )
            )
            (ExecutionOutputLink
               (GroundedSchemaNode "scm: conditional-full-instantiation-scope-formula")
               (ListLink
                  (InheritanceLink
                     (ConceptNode "a")
                     (ConceptNode "g")
                  )
                  (EvaluationLink (stv 1 1)
                     (PredicateNode "alphabetical-order")
                     (ListLink
                        (ConceptNode "a")
                        (ConceptNode "g")
                     )
                  )
                  (ImplicationScopeLink (stv 1 1)
                     (VariableList
                        (TypedVariableLink
                           (VariableNode "$X")
                           (TypeNode "ConceptNode")
                        )
                        (TypedVariableLink
                           (VariableNode "$Y")
                           (TypeNode "ConceptNode")
                        )
                     )
                     (EvaluationLink
                        (PredicateNode "alphabetical-order")
                        (ListLink
                           (VariableNode "$X")
                           (VariableNode "$Y")
                        )
                     )
                     (InheritanceLink
                        (VariableNode "$X")
                        (VariableNode "$Y")
                     )
                  )
               )
            )
         )
      )
      (InheritanceLink
         (ConceptNode "a")
         (ConceptNode "g")
      )
   )
)
(ExecutionLink (stv 1 1)
   (SchemaNode "URE:BC:expand-and-BIT")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "a")
               (ConceptNode "g")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "a")
         (ConceptNode "g")
      )
      (DontExecLink
         (DefinedSchemaNode "modus-ponens-subset-rule")
      )
   )
   (DontExecLink
      (BindLink
         (VariableNode "$A-3ce0ef76")
         (AndLink
            (VariableNode "$A-3ce0ef76")
            (SubsetLink
               (VariableNode "$A-3ce0ef76")
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "g")
               )
            )
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (VariableNode "$A-3ce0ef76")
            )
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (SubsetLink
                  (VariableNode "$A-3ce0ef76")
                  (InheritanceLink
                     (ConceptNode "a")
                     (ConceptNode "g")
                  )
               )
            )
         )
         (ExecutionOutputLink
            (GroundedSchemaNode "scm: modus-ponens-formula")
            (ListLink
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "g")
               )
               (SubsetLink
                  (VariableNode "$A-3ce0ef76")
                  (InheritanceLink
                     (ConceptNode "a")
                     (ConceptNode "g")
                  )
               )
               (VariableNode "$A-3ce0ef76")
            )
         )
      )
   )
)
(ExecutionLink (stv 1 1)
   (SchemaNode "URE:BC:expand-and-BIT")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
               (EvaluationLink
                  (GroundedPredicateNode "scm: gt-zero-confidence")
                  (EvaluationLink
                     (PredicateNode "alphabetical-order")
                     (ListLink
                        (ConceptNode "d")
                        (ConceptNode "y")
                     )
                  )
               )
            )
            (ExecutionOutputLink
               (GroundedSchemaNode "scm: conditional-full-instantiation-scope-formula")
               (ListLink
                  (InheritanceLink
                     (ConceptNode "d")
                     (ConceptNode "y")
                  )
                  (EvaluationLink
                     (PredicateNode "alphabetical-order")
                     (ListLink
                        (ConceptNode "d")
                        (ConceptNode "y")
                     )
                  )
                  (ImplicationScopeLink (stv 1 1)
                     (VariableList
                        (TypedVariableLink
                           (VariableNode "$X")
                           (TypeNode "ConceptNode")
                        )
                        (TypedVariableLink
                           (VariableNode "$Y")
                           (TypeNode "ConceptNode")
                        )
                     )
                     (EvaluationLink
                        (PredicateNode "alphabetical-order")
                        (ListLink
                           (VariableNode "$X")
                           (VariableNode "$Y")
                        )
                     )
                     (InheritanceLink
                        (VariableNode "$X")
                        (VariableNode "$Y")
                     )
                  )
               )
            )
         )
      )
      (EvaluationLink
         (PredicateNode "alphabetical-order")
         (ListLink
            (ConceptNode "d")
            (ConceptNode "y")
         )
      )
      (DontExecLink
         (DefinedSchemaNode "modus-ponens-implication-rule")
      )
   )
   (DontExecLink
      (BindLink
         (VariableNode "$A-4d3401d5")
         (AndLink
            (VariableNode "$A-4d3401d5")
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (VariableNode "$A-4d3401d5")
            )
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (ImplicationLink
                  (VariableNode "$A-4d3401d5")
                  (EvaluationLink
                     (PredicateNode "alphabetical-order")
                     (ListLink
                        (ConceptNode "d")
                        (ConceptNode "y")
                     )
                  )
               )
            )
            (ImplicationLink
               (VariableNode "$A-4d3401d5")
               (EvaluationLink
                  (PredicateNode "alphabetical-order")
                  (ListLink
                     (ConceptNode "d")
                     (ConceptNode "y")
                  )
               )
            )
         )
         (ExecutionOutputLink
            (GroundedSchemaNode "scm: conditional-full-instantiation-scope-formula")
            (ListLink
               (InheritanceLink
                  (ConceptNode "d")
                  (ConceptNode "y")
               )
               (ExecutionOutputLink
                  (GroundedSchemaNode "scm: modus-ponens-formula")
                  (ListLink
                     (EvaluationLink
                        (PredicateNode "alphabetical-order")
                        (ListLink
                           (ConceptNode "d")
                           (ConceptNode "y")
                        )
                     )
                     (ImplicationLink
                        (VariableNode "$A-4d3401d5")
                        (EvaluationLink
                           (PredicateNode "alphabetical-order")
                           (ListLink
                              (ConceptNode "d")
                              (ConceptNode "y")
                           )
                        )
                     )
                     (VariableNode "$A-4d3401d5")
                  )
               )
               (ImplicationScopeLink (stv 1 1)
                  (VariableList
                     (TypedVariableLink
                        (VariableNode "$X")
                        (TypeNode "ConceptNode")
                     )
                     (TypedVariableLink
                        (VariableNode "$Y")
                        (TypeNode "ConceptNode")
                     )
                  )
                  (EvaluationLink
                     (PredicateNode "alphabetical-order")
                     (ListLink
                        (VariableNode "$X")
                        (VariableNode "$Y")
                     )
                  )
                  (InheritanceLink
                     (VariableNode "$X")
                     (VariableNode "$Y")
                  )
               )
            )
         )
      )
   )
)
(ExecutionLink (stv 1 1)
   (SchemaNode "URE:BC:expand-and-BIT")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "a")
               (ConceptNode "g")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "a")
         (ConceptNode "g")
      )
      (DontExecLink
         (DefinedSchemaNode "modus-ponens-implication-rule")
      )
   )
   (DontExecLink
      (BindLink
         (VariableNode "$A-485f010a")
         (AndLink
            (VariableNode "$A-485f010a")
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (ImplicationLink
                  (VariableNode "$A-485f010a")
                  (InheritanceLink
                     (ConceptNode "a")
                     (ConceptNode "g")
                  )
               )
            )
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (VariableNode "$A-485f010a")
            )
            (ImplicationLink
               (VariableNode "$A-485f010a")
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "g")
               )
            )
         )
         (ExecutionOutputLink
            (GroundedSchemaNode "scm: modus-ponens-formula")
            (ListLink
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "g")
               )
               (ImplicationLink
                  (VariableNode "$A-485f010a")
                  (InheritanceLink
                     (ConceptNode "a")
                     (ConceptNode "g")
                  )
               )
               (VariableNode "$A-485f010a")
            )
         )
      )
   )
)
(ExecutionLink (stv 1 1)
   (SchemaNode "URE:BC:expand-and-BIT")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "a")
               (ConceptNode "z")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "a")
         (ConceptNode "z")
      )
      (DontExecLink
         (DefinedSchemaNode "modus-ponens-subset-rule")
      )
   )
   (DontExecLink
      (BindLink
         (VariableNode "$A-42e1ada")
         (AndLink
            (VariableNode "$A-42e1ada")
            (SubsetLink
               (VariableNode "$A-42e1ada")
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "z")
               )
            )
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (SubsetLink
                  (VariableNode "$A-42e1ada")
                  (InheritanceLink
                     (ConceptNode "a")
                     (ConceptNode "z")
                  )
               )
            )
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (VariableNode "$A-42e1ada")
            )
         )
         (ExecutionOutputLink
            (GroundedSchemaNode "scm: modus-ponens-formula")
            (ListLink
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "z")
               )
               (SubsetLink
                  (VariableNode "$A-42e1ada")
                  (InheritanceLink
                     (ConceptNode "a")
                     (ConceptNode "z")
                  )
               )
               (VariableNode "$A-42e1ada")
            )
         )
      )
   )
)
(ExecutionLink (stv 1 1)
   (SchemaNode "URE:BC:expand-and-BIT")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "a")
               (ConceptNode "z")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "a")
         (ConceptNode "z")
      )
      (DontExecLink
         (DefinedSchemaNode "conditional-full-instantiation-implication-scope-meta-rule")
      )
   )
   (DontExecLink
      (BindLink
         (AndLink
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (EvaluationLink (stv 1 1)
                  (PredicateNode "alphabetical-order")
                  (ListLink
                     (ConceptNode "a")
                     (ConceptNode "z")
                  )
               )
            )
         )
         (ExecutionOutputLink
            (GroundedSchemaNode "scm: conditional-full-instantiation-scope-formula")
            (ListLink
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "z")
               )
               (EvaluationLink (stv 1 1)
                  (PredicateNode "alphabetical-order")
                  (ListLink
                     (ConceptNode "a")
                     (ConceptNode "z")
                  )
               )
               (ImplicationScopeLink (stv 1 1)
                  (VariableList
                     (TypedVariableLink
                        (VariableNode "$X")
                        (TypeNode "ConceptNode")
                     )
                     (TypedVariableLink
                        (VariableNode "$Y")
                        (TypeNode "ConceptNode")
                     )
                  )
                  (EvaluationLink
                     (PredicateNode "alphabetical-order")
                     (ListLink
                        (VariableNode "$X")
                        (VariableNode "$Y")
                     )
                  )
                  (InheritanceLink
                     (VariableNode "$X")
                     (VariableNode "$Y")
                  )
               )
            )
         )
      )
   )
)
(ExecutionLink (stv 1 1)
   (SchemaNode "URE:BC:expand-and-BIT")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "a")
               (ConceptNode "z")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "a")
         (ConceptNode "z")
      )
      (DontExecLink
         (DefinedSchemaNode "modus-ponens-inheritance-rule")
      )
   )
   (DontExecLink
      (BindLink
         (VariableNode "$A-2a3abe9e")
         (AndLink
            (VariableNode "$A-2a3abe9e")
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (VariableNode "$A-2a3abe9e")
            )
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (InheritanceLink
                  (VariableNode "$A-2a3abe9e")
                  (InheritanceLink
                     (ConceptNode "a")
                     (ConceptNode "z")
                  )
               )
            )
            (InheritanceLink
               (VariableNode "$A-2a3abe9e")
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "z")
               )
            )
         )
         (ExecutionOutputLink
            (GroundedSchemaNode "scm: modus-ponens-formula")
            (ListLink
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "z")
               )
               (InheritanceLink
                  (VariableNode "$A-2a3abe9e")
                  (InheritanceLink
                     (ConceptNode "a")
                     (ConceptNode "z")
                  )
               )
               (VariableNode "$A-2a3abe9e")
            )
         )
      )
   )
)
(ExecutionLink (stv 1 1)
   (SchemaNode "URE:BC:expand-and-BIT")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "d")
               (ConceptNode "y")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "d")
         (ConceptNode "y")
      )
      (DontExecLink
         (DefinedSchemaNode "conditional-full-instantiation-implication-scope-meta-rule")
      )
   )
   (DontExecLink
      (BindLink
         (AndLink
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (EvaluationLink
                  (PredicateNode "alphabetical-order")
                  (ListLink
                     (ConceptNode "d")
                     (ConceptNode "y")
                  )
               )
            )
         )
         (ExecutionOutputLink
            (GroundedSchemaNode "scm: conditional-full-instantiation-scope-formula")
            (ListLink
               (InheritanceLink
                  (ConceptNode "d")
                  (ConceptNode "y")
               )
               (EvaluationLink
                  (PredicateNode "alphabetical-order")
                  (ListLink
                     (ConceptNode "d")
                     (ConceptNode "y")
                  )
               )
               (ImplicationScopeLink (stv 1 1)
                  (VariableList
                     (TypedVariableLink
                        (VariableNode "$X")
                        (TypeNode "ConceptNode")
                     )
                     (TypedVariableLink
                        (VariableNode "$Y")
                        (TypeNode "ConceptNode")
                     )
                  )
                  (EvaluationLink
                     (PredicateNode "alphabetical-order")
                     (ListLink
                        (VariableNode "$X")
                        (VariableNode "$Y")
                     )
                  )
                  (InheritanceLink
                     (VariableNode "$X")
                     (VariableNode "$Y")
                  )
               )
            )
         )
      )
   )
)
(ExecutionLink (stv 1 1)
   (SchemaNode "URE:BC:expand-and-BIT")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "a")
               (ConceptNode "z")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "a")
         (ConceptNode "z")
      )
      (DontExecLink
         (DefinedSchemaNode "deduction-inheritance-rule")
      )
   )
   (DontExecLink
      (BindLink
         (TypedVariableLink
            (VariableNode "$B-21212abe")
            (TypeChoice
               (TypeNode "OrLink")
               (TypeNode "ConceptNode")
               (TypeNode "AndLink")
               (TypeNode "NotLink")
            )
         )
         (AndLink
            (NotLink
               (IdenticalLink
                  (ConceptNode "z")
                  (ConceptNode "a")
               )
            )
            (InheritanceLink
               (ConceptNode "a")
               (VariableNode "$B-21212abe")
            )
            (InheritanceLink
               (VariableNode "$B-21212abe")
               (ConceptNode "z")
            )
         )
         (ExecutionOutputLink
            (GroundedSchemaNode "scm: deduction-formula")
            (ListLink
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "z")
               )
               (InheritanceLink
                  (ConceptNode "a")
                  (VariableNode "$B-21212abe")
               )
               (InheritanceLink
                  (VariableNode "$B-21212abe")
                  (ConceptNode "z")
               )
            )
         )
      )
   )
)
(ExecutionLink (stv 1 1)
   (SchemaNode "URE:BC:expand-and-BIT")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "a")
               (ConceptNode "g")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "a")
         (ConceptNode "g")
      )
      (DontExecLink
         (DefinedSchemaNode "modus-ponens-inheritance-rule")
      )
   )
   (DontExecLink
      (BindLink
         (VariableNode "$A-51055a30")
         (AndLink
            (VariableNode "$A-51055a30")
            (InheritanceLink
               (VariableNode "$A-51055a30")
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "g")
               )
            )
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (VariableNode "$A-51055a30")
            )
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (InheritanceLink
                  (VariableNode "$A-51055a30")
                  (InheritanceLink
                     (ConceptNode "a")
                     (ConceptNode "g")
                  )
               )
            )
         )
         (ExecutionOutputLink
            (GroundedSchemaNode "scm: modus-ponens-formula")
            (ListLink
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "g")
               )
               (InheritanceLink
                  (VariableNode "$A-51055a30")
                  (InheritanceLink
                     (ConceptNode "a")
                     (ConceptNode "g")
                  )
               )
               (VariableNode "$A-51055a30")
            )
         )
      )
   )
)
(ExecutionLink (stv 1 1)
   (SchemaNode "URE:BC:expand-and-BIT")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "d")
               (ConceptNode "y")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "d")
         (ConceptNode "y")
      )
      (DontExecLink
         (DefinedSchemaNode "modus-ponens-subset-rule")
      )
   )
   (DontExecLink
      (BindLink
         (VariableNode "$A-2df312ff")
         (AndLink
            (VariableNode "$A-2df312ff")
            (SubsetLink
               (VariableNode "$A-2df312ff")
               (InheritanceLink
                  (ConceptNode "d")
                  (ConceptNode "y")
               )
            )
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (SubsetLink
                  (VariableNode "$A-2df312ff")
                  (InheritanceLink
                     (ConceptNode "d")
                     (ConceptNode "y")
                  )
               )
            )
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (VariableNode "$A-2df312ff")
            )
         )
         (ExecutionOutputLink
            (GroundedSchemaNode "scm: modus-ponens-formula")
            (ListLink
               (InheritanceLink
                  (ConceptNode "d")
                  (ConceptNode "y")
               )
               (SubsetLink
                  (VariableNode "$A-2df312ff")
                  (InheritanceLink
                     (ConceptNode "d")
                     (ConceptNode "y")
                  )
               )
               (VariableNode "$A-2df312ff")
            )
         )
      )
   )
)
(ExecutionLink (stv 1 1)
   (SchemaNode "URE:BC:expand-and-BIT")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "a")
               (ConceptNode "g")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "a")
         (ConceptNode "g")
      )
      (DontExecLink
         (DefinedSchemaNode "conditional-full-instantiation-implication-scope-meta-rule")
      )
   )
   (DontExecLink
      (BindLink
         (AndLink
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (EvaluationLink (stv 1 1)
                  (PredicateNode "alphabetical-order")
                  (ListLink
                     (ConceptNode "a")
                     (ConceptNode "g")
                  )
               )
            )
         )
         (ExecutionOutputLink
            (GroundedSchemaNode "scm: conditional-full-instantiation-scope-formula")
            (ListLink
               (InheritanceLink
                  (ConceptNode "a")
                  (ConceptNode "g")
               )
               (EvaluationLink (stv 1 1)
                  (PredicateNode "alphabetical-order")
                  (ListLink
                     (ConceptNode "a")
                     (ConceptNode "g")
                  )
               )
               (ImplicationScopeLink (stv 1 1)
                  (VariableList
                     (TypedVariableLink
                        (VariableNode "$X")
                        (TypeNode "ConceptNode")
                     )
                     (TypedVariableLink
                        (VariableNode "$Y")
                        (TypeNode "ConceptNode")
                     )
                  )
                  (EvaluationLink
                     (PredicateNode "alphabetical-order")
                     (ListLink
                        (VariableNode "$X")
                        (VariableNode "$Y")
                     )
                  )
                  (InheritanceLink
                     (VariableNode "$X")
                     (VariableNode "$Y")
                  )
               )
            )
         )
      )
   )
)
(ExecutionLink (stv 1 1)
   (SchemaNode "URE:BC:expand-and-BIT")
   (ListLink
      (DontExecLink
         (BindLink
            (AndLink
            )
            (InheritanceLink
               (ConceptNode "d")
               (ConceptNode "y")
            )
         )
      )
      (InheritanceLink
         (ConceptNode "d")
         (ConceptNode "y")
      )
      (DontExecLink
         (DefinedSchemaNode "modus-ponens-inheritance-rule")
      )
   )
   (DontExecLink
      (BindLink
         (VariableNode "$A-5e64c5c0")
         (AndLink
            (VariableNode "$A-5e64c5c0")
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (InheritanceLink
                  (VariableNode "$A-5e64c5c0")
                  (InheritanceLink
                     (ConceptNode "d")
                     (ConceptNode "y")
                  )
               )
            )
            (EvaluationLink
               (GroundedPredicateNode "scm: gt-zero-confidence")
               (VariableNode "$A-5e64c5c0")
            )
            (InheritanceLink
               (VariableNode "$A-5e64c5c0")
               (InheritanceLink
                  (ConceptNode "d")
                  (ConceptNode "y")
               )
            )
         )
         (ExecutionOutputLink
            (GroundedSchemaNode "scm: modus-ponens-formula")
            (ListLink
               (InheritanceLink
                  (ConceptNode "d")
                  (ConceptNode "y")
               )
               (InheritanceLink
                  (VariableNode "$A-5e64c5c0")
                  (InheritanceLink
                     (ConceptNode "d")
                     (ConceptNode "y")
                  )
               )
               (VariableNode "$A-5e64c5c0")
            )
         )
      )
   )
)
