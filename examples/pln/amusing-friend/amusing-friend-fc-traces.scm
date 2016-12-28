;; Print the inference steps leading to this target

;; Just paste the following 2 functions, then paste the steps you
;; would like to query (only the target queries are given)
(define (get-target-inference-steps target)
  (let* ((pat (Execution
                 (Variable "$rule")
                 (Variable "$iteration")
                 (Variable "$Source")
                 target))
         (gl (Get pat)))
    (List (cog-execute! gl) target (Number (cog-handle target)))))

(define (get-source-inference-steps source)
  (let* ((pat (Execution
                 (Variable "$rule")
                 (Variable "$iteration")
                 source
                 (Variable "$target")))
         (gl (Get pat)))
    (List (cog-execute! gl) source (Number (cog-handle source)))))

;; (1)
(define target-1 (Evaluation (Predicate "is-honest") (Concept "Bob")))
(get-target-inference-steps target-1)

;; (2)
(define target-2 (Implication
   (Lambda
      (VariableList
         (TypedVariable
            (Variable "$X")
            (Type "ConceptNode"))
         (TypedVariable
            (Variable "$Y")
            (Type "ConceptNode")))
      (Evaluation
         (Predicate "will-be-friends")
         (List
            (Variable "$X")
            (Variable "$Y"))))
   (Lambda
      (VariableList
         (TypedVariable
            (Variable "$X")
            (Type "ConceptNode"))
         (TypedVariable
            (Variable "$Y")
            (Type "ConceptNode")))
      (And
         (Evaluation
            (Predicate "is-honest")
            (Variable "$X"))
         (Evaluation
            (Predicate "is-honest")
            (Variable "$Y"))))))
(get-target-inference-steps target-2)

;; (3)
(define target-3 (Lambda
   (VariableList
      (TypedVariable
         (Variable "$X")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$Y")
         (Type "ConceptNode")))
   (Evaluation
      (Predicate "will-be-friends")
      (List
         (Variable "$X")
         (Variable "$Y")))))
(get-target-inference-steps target-3)

;; (5)
(define target-5 (Implication
   (Lambda
      (VariableList
         (TypedVariable
            (Variable "$X")
            (Type "ConceptNode"))
         (TypedVariable
            (Variable "$Y")
            (Type "ConceptNode")))
      (And
         (Evaluation
            (Predicate "is-honest")
            (Variable "$X"))
         (Evaluation
            (Predicate "is-honest")
            (Variable "$Y"))))
   (Lambda
      (VariableList
         (TypedVariable
            (Variable "$X")
            (Type "ConceptNode"))
         (TypedVariable
            (Variable "$Y")
            (Type "ConceptNode")))
      (Evaluation
         (Predicate "will-be-friends")
         (List
            (Variable "$X")
            (Variable "$Y"))))))
(get-target-inference-steps target-5)
  
;; (6)
(define target-6
(Implication
   (LambdaLink
      (VariableList
         (TypedVariable
            (Variable "$X")
            (Type "ConceptNode"))
         (TypedVariable
            (Variable "$Y")
            (Type "ConceptNode")))
      (And
         (Inheritance
            (Variable "$X")
            (Concept "human"))
         (Inheritance
            (Variable "$Y")
            (Concept "human"))
         (Evaluation
            (Predicate "acquainted")
            (List
               (Variable "$X")
               (Variable "$Y")))))
   (LambdaLink
      (VariableList
         (TypedVariable
            (Variable "$X")
            (Type "ConceptNode"))
         (TypedVariable
            (Variable "$Y")
            (Type "ConceptNode")))
      (Evaluation
         (Predicate "will-be-friends")
         (List
            (Variable "$X")
            (Variable "$Y"))))))
(get-target-inference-steps target-6)

;; (7)
(define target-7
(ImplicationLink
   (AndLink
      (LambdaLink
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
         (AndLink
            (EvaluationLink
               (PredicateNode "is-honest")
               (VariableNode "$X")
            )
            (EvaluationLink
               (PredicateNode "is-honest")
               (VariableNode "$Y")
            )
         )
      )
      (LambdaLink
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
         (AndLink
            (InheritanceLink
               (VariableNode "$X")
               (ConceptNode "human")
            )
            (InheritanceLink
               (VariableNode "$Y")
               (ConceptNode "human")
            )
            (EvaluationLink
               (PredicateNode "acquainted")
               (ListLink
                  (VariableNode "$X")
                  (VariableNode "$Y")
               )
            )
         )
      )
   )
   (LambdaLink
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
         (PredicateNode "will-be-friends")
         (ListLink
            (VariableNode "$X")
            (VariableNode "$Y")
         )
      )
   )
)
)
(get-target-inference-steps target-7)

;; (8)
(define target-8
(ImplicationLink
   (LambdaLink
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
      (AndLink
         (EvaluationLink
            (PredicateNode "is-honest")
            (VariableNode "$X")
         )
         (EvaluationLink
            (PredicateNode "is-honest")
            (VariableNode "$Y")
         )
         (InheritanceLink
            (VariableNode "$X")
            (ConceptNode "human")
         )
         (InheritanceLink
            (VariableNode "$Y")
            (ConceptNode "human")
         )
         (EvaluationLink
            (PredicateNode "acquainted")
            (ListLink
               (VariableNode "$X")
               (VariableNode "$Y")
            )
         )
      )
   )
   (AndLink
      (LambdaLink
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
         (AndLink
            (EvaluationLink
               (PredicateNode "is-honest")
               (VariableNode "$X")
            )
            (EvaluationLink
               (PredicateNode "is-honest" (stv 0.80000001 0.89999998))
               (VariableNode "$Y")
            )
         )
      )
      (LambdaLink
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
         (AndLink
            (InheritanceLink
               (VariableNode "$X")
               (ConceptNode "human")
            )
            (InheritanceLink
               (VariableNode "$Y")
               (ConceptNode "human")
            )
            (EvaluationLink
               (PredicateNode "acquainted")
               (ListLink
                  (VariableNode "$X")
                  (VariableNode "$Y")
               )
            )
         )
      )
   )
)
)
(get-target-inference-steps target-8)

;; (9)
(define target-9
(ImplicationLink
   (LambdaLink
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
      (AndLink
         (EvaluationLink
            (PredicateNode "is-honest")
            (VariableNode "$X")
         )
         (EvaluationLink
            (PredicateNode "is-honest")
            (VariableNode "$Y")
         )
         (InheritanceLink
            (VariableNode "$X")
            (ConceptNode "human")
         )
         (InheritanceLink
            (VariableNode "$Y")
            (ConceptNode "human")
         )
         (EvaluationLink
            (PredicateNode "acquainted")
            (ListLink
               (VariableNode "$X")
               (VariableNode "$Y")
            )
         )
      )
   )
   (LambdaLink
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
         (PredicateNode "will-be-friends")
         (ListLink
            (VariableNode "$X")
            (VariableNode "$Y")
         )
      )
   )
)
)
(get-target-inference-steps target-9)

;; (10)
(define target-10
(ImplicationScopeLink
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
   (AndLink
      (EvaluationLink
         (PredicateNode "is-honest")
         (VariableNode "$X")
      )
      (EvaluationLink
         (PredicateNode "is-honest")
         (VariableNode "$Y")
      )
      (InheritanceLink
         (VariableNode "$X")
         (ConceptNode "human")
      )
      (InheritanceLink
         (VariableNode "$Y")
         (ConceptNode "human")
      )
      (EvaluationLink
         (PredicateNode "acquainted")
         (ListLink
            (VariableNode "$X")
            (VariableNode "$Y")
         )
      )
   )
   (EvaluationLink
      (PredicateNode "will-be-friends")
      (ListLink
         (VariableNode "$X")
         (VariableNode "$Y")
      )
   )
)
)
(get-target-inference-steps target-10)

;; (11)
(define target-11
(EvaluationLink
   (PredicateNode "will-be-friends")
   (ListLink
      (ConceptNode "Self")
      (ConceptNode "Bob")
   )
)
)
(get-target-inference-steps target-11)

;; (12)
(define target-12
(Evaluation
  (Predicate "is-funny")
  (Concept "Bob"))
)
(get-target-inference-steps target-12)

;; (13)
(define target-13
(Equivalence
   (Lambda
      (TypedVariable
         (Variable "$X")
         (Type "ConceptNode"))
      (Evaluation
         (Predicate "is-funny")
         (Variable "$X")))
   (Lambda
      (TypedVariable
         (Variable "$X")
         (Type "ConceptNode"))
      (Evaluation
         (Predicate "is-amusing")
         (Variable "$X")))))
(get-target-inference-steps target-13)

;; (14)
(define target-14
(Implication
   (Lambda
      (TypedVariable
         (Variable "$X")
         (Type "ConceptNode"))
      (Evaluation
         (Predicate "is-funny")
         (Variable "$X")))
   (Lambda
      (TypedVariable
         (Variable "$X")
         (Type "ConceptNode"))
      (Evaluation
         (Predicate "is-amusing")
         (Variable "$X")))))
(get-target-inference-steps target-14)

;; (15)
(define target-15
(ImplicationScopeLink
   (TypedVariableLink
      (VariableNode "$X")
      (TypeNode "ConceptNode")
   )
   (EvaluationLink
      (PredicateNode "is-funny")
      (VariableNode "$X")
   )
   (EvaluationLink
      (PredicateNode "is-amusing")
      (VariableNode "$X")
   )
)
)
(get-target-inference-steps target-15)

;; (16)
(define target-16
(Evaluation
  (Predicate "is-amusing")
  (Concept "Bob")))
(get-target-inference-steps target-16)

;; (17)
(define target-17
(And
   (Evaluation
      (Predicate "will-be-friends")
      (List
         (Concept "Self")
         (Concept "Bob")))
   (Evaluation
      (Predicate "is-amusing")
      (Concept "Bob"))
   (Evaluation
      (Predicate "is-honest")
      (Concept "Bob"))))
(get-target-inference-steps target-17)
