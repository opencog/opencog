;; Kownledge base for the amusing friend demo.
;;
;; We are dodging a lot of representational issues here. In particular
;; everything related to spatio-temporal reasoning. To be reified as
;; desired.

;;;;;;;;;;;;;
;; Honesty ;;
;;;;;;;;;;;;;

;; Probability of being honest
(Predicate "is-honest" (stv 0.8 0.9))

;; Probability that two things are honest
;;
;; This should be inferred since we don't have the rules to infer that
;; we put it in the kb for now.
(Lambda (stv 0.64 0.9)
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

;; Probability of telling the truth to someone. The probability if
;; very low cause the probability of telling something to someone is
;; already very low.
(Predicate "told-the-truth" (stv 0.00001 0.7))

;; We need also the following. It should normally be wrapped in a
;; Lambda, but because instantiation. And ultimately this should be
;; inferred.
(Evaluation (stv 0.00001 0.7)
   (Predicate "told-the-truth-about")
   (List
      (Variable "$X")
      (Variable "$Y")
      (Variable "$Z")))

;; People who told the truth about something are honest
(define people-telling-the-truth-are-honest
(ImplicationScope (stv 0.95 0.9)
   (VariableList
      (TypedVariable
         (Variable "$X")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$Y")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$Z")
         (Type "ConceptNode")))
   (Evaluation
      (Predicate "told-the-truth-about")
      (List
         (Variable "$X")
         (Variable "$Y")
         (Variable "$Z")))
   (Evaluation
      (Predicate "is-honest")
      (Variable "$X"))))

;;;;;;;;;;;;;;
;; Humanity ;;
;;;;;;;;;;;;;;

;; Probability of two human acquaintances
(Lambda (stv 0.0002 0.9)
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

;;;;;;;;;
;; Bob ;;
;;;;;;;;;

;; Bob is a human
(Inheritance (stv 1 1)
   (Concept "Bob")
   (Concept "human"))

;;;;;;;;;;
;; Self ;;
;;;;;;;;;;

;; I am a human
(Inheritance (stv 1 1)
   (Concept "Self")
   (Concept "human"))

;; I am honest
(Evaluation (stv 0.9 0.9)
   (Predicate "is-honest")
   (Concept "Self"))

;; I know Bob
(Evaluation (stv 1 1)
   (Predicate "acquainted")
   (List
      (Concept "Self")
      (Concept "Bob")))

;;;;;;;;;;;;;;;;
;; Friendship ;;
;;;;;;;;;;;;;;;;

;; The probability of random things (typically humans) being friends
(Predicate "will-be-friends" (stv 0.0001 0.9))

;; Because we have no way to specify the type signature of predicate
;; "will-be-friends" (will need a better type system) we specify it
;; indirectly by wrapping it in a lambda. The TV on the lambda can be
;; evaluated by inference but the structure cannot.
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

;; Friendship is symmetric
(ImplicationScope (stv 1 1)
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
         (Variable "$Y")))
   (Evaluation
      (Predicate "will-be-friends")
      (List
         (Variable "$Y")
         (Variable "$X"))))

;; I'm disabling that to simplify the inference. Ultimately the only
;; reason we use will-be-friends rather than are-friends is so the
;; first person perspective makes a bit of sense (cause someone is
;; supposed to know who are her friends). With a third person
;; perspective, such as "Find Sylvia's friends", then we can just use
;; "are-friends", cause we're not supposed to know all of Sylvia's
;; friends.
;;
;; ;; Friends will remain friends.
;; ;;
;; ;; This could simply be expressed as
;; ;;
;; ;; (Implication (stv 0.9 0.9)
;; ;;    (Predicate "are-friends")
;; ;;    (Predicate "will-be-friends"))
;; ;;
;; ;; but due to some current limitation in the type system, specifically
;; ;; that a Predicate cannot be declared with a certain type, we need to
;; ;; express that in a more convoluted way.
;; (ImplicationScope (stv 0.9 0.9)
;;    (VariableList
;;       (TypedVariable
;;          (Variable "$X")
;;          (Type "ConceptNode"))
;;       (TypedVariable
;;          (Variable "$Y")
;;          (Type "ConceptNode")))
;;    (Evaluation
;;       (Predicate "are-friends")
;;       (List
;;          (Variable "$X")
;;          (Variable "$Y")))
;;    (Evaluation
;;       (Predicate "will-be-friends")
;;       (List
;;          (Variable "$X")
;;          (Variable "$Y"))))

;; The probablity of turning acquaintance into friendship between
;; humans is 0.1.
(define human-acquainted-tend-to-become-friends
(ImplicationScope (stv 0.1 0.5)
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
            (Variable "$Y"))))
   (Evaluation
      (Predicate "will-be-friends")
      (List
         (Variable "$X")
         (Variable "$Y")))))

;; Friends tend to be honest
(define friends-tend-to-be-honest
(ImplicationScope (stv 0.85 0.5)
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
         (Variable "$Y")))
   (And
      (Evaluation
         (Predicate "is-honest")
         (Variable "$X"))
      (Evaluation
         (Predicate "is-honest")
         (Variable "$Y")))))

;;;;;;;;;;;;;;;;;
;; Being Funny ;;
;;;;;;;;;;;;;;;;;

;; Probability of telling a joke to someone. The probability is
;; extremely low because the probability of telling anything to
;; someone is already very low.
(Predicate "told-a-joke-at" (stv 0.000001 0.6))

;; The following should be wrapped in a Lambda and ultimately
;; inferred.
(Evaluation (stv 0.000001 0.6)
   (Predicate "told-a-joke-at")
      (List
         (Variable "$X")
         (Variable "$Y")
         (Variable "$Z")))

;; Probability of being funny
(Predicate "is-funny" (stv 0.69 0.7))

;; Same remark as for Predicate "told-a-joke-at"
(Evaluation (stv 0.69 0.7)
   (Predicate "is-funny")
   (Variable "$X"))

;; People who told a joke to someone, somewhere, are funny   
(define people-telling-jokes-are-funny
(ImplicationScope (stv 0.8 0.9)
   (VariableList
      (TypedVariable
         (Variable "$X")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$Y")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$Z")
         (Type "ConceptNode")))
   (Evaluation
      (Predicate "told-a-joke-at")
      (List
         (Variable "$X")
         (Variable "$Y")
         (Variable "$Z")))
   (Evaluation
      (Predicate "is-funny")
      (Variable "$X"))))

;; Being funny is loosely equivalent to being amusing
;;
;; This could simply be expressed as
;;
;; (Equivalence (stv 0.7 0.9)
;;    (Predicate "is-funny")
;;    (Predicate "is-amusing"))
;;
;; but due to some current limitation in the type system, specifically
;; that a Predicate cannot be declared with a certain type, we need to
;; express that in a more convoluted way.
(define funny-is-loosely-equivalent-to-amusing
(Equivalence (stv 0.7 0.9)
   (TypedVariable
      (Variable "$X")
      (Type "ConceptNode"))
   (Evaluation
      (Predicate "is-funny")
      (Variable "$X"))
   (Evaluation
      (Predicate "is-amusing")
      (Variable "$X"))))

;;;;;;;;;;;;;;;
;; The Party ;;
;;;;;;;;;;;;;;;

;; Bob told Jill the truth about the party
(Evaluation (stv 1 1)
   (Predicate "told-the-truth-about")
   (List
      (Concept "Bob")
      (Concept "Jill")
      (Concept "Party")))


;; Bob told Jim a joke at the party.
(Evaluation (stv 1 1)
   (Predicate "told-a-joke-at")
      (List
         (Concept "Bob")
         (Concept "Jim")
         (Concept "Party")))

;;;;;;;;;;
;; Hack ;;
;;;;;;;;;;

;; Due to the fact the evaluator does not support fuzzy TV semantic we
;; put the evaluation of a to-be-used instantiated precondition
;; here. Alternatively we could add PLN rules to evaluate.
(define hack (And (stv 1 0.9)
   (Evaluation
      (Predicate "is-honest")
      (Concept "Self")
   )
   (Evaluation
      (Predicate "is-honest")
      (Concept "Bob")
   )
   (Inheritance
      (Concept "Self")
      (Concept "human")
   )
   (Inheritance
      (Concept "Bob")
      (Concept "human")
   )
   (Evaluation
      (Predicate "acquainted")
      (ListLink
         (Concept "Self")
         (Concept "Bob")
      )
   )
)
)

;; Because implication-instantiation occurs on the sugar syntax, the
;; predicate (which should be wrapped in a Lambda) is not given. Also
;; of course that predicate should still be evaluated. Here we provide
;; the adequate TV value of that predicate on the free scope form.
(AndLink (stv 0.000128 0.89999998)
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
