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

;;;;;;;;;;;;;;
;; Humanity ;;
;;;;;;;;;;;;;;

;; Probability of two human acquaintances
(Lambda (stv 0.001 0.9)
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

;; Friendship is symmetric
(Implication (stv 1 1)
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
;; (Implication (stv 0.9 0.9)
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
(Implication (stv 0.1 0.5)
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
         (Variable "$Y"))))

;; Friends tend to be honest
(Implication (stv 0.75 0.5)
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
         (Variable "$Y"))))

;;;;;;;;;;;;;;;;;;;;;;;
;; Telling the truth ;;
;;;;;;;;;;;;;;;;;;;;;;;

;; People who told the truth about something are honest
(Implication (stv 0.8 0.9)
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
      (Variable "$X")))

;;;;;;;;;;;;;;;;;
;; Being Funny ;;
;;;;;;;;;;;;;;;;;

;; People who told a joke to someone, somewhere, are funny   
(Implication (stv 0.8 0.9)
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
      (Variable "$X")))

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
(Equivalence (stv 0.7 0.9)
   (TypedVariable
      (Variable "$X")
      (Type "ConceptNode"))
   (Evaluation
      (Predicate "is-funny")
      (Variable "$X"))
   (Evaluation
      (Predicate "is-amusing")
      (Variable "$X")))

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
