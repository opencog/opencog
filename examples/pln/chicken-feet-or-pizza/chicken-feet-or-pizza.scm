;; Inference example combining the openpsi chatbot and PLN

(add-to-load-path "../../../opencog/nlp/chatbot-psi")
(load-from-path "chatbot.scm")

(use-modules (opencog query))
(define (get-parse-nodes)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$P")
                           (Type "ParseNode"))
                         (Variable "$P"))))

(define (get-set-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$S")
                           (Type "SetLink"))
                         (Variable "$S"))))

;; parse-get-r2l-outputs

;; “I’m Bob.   I live in China.   I like chicken feet.”
;; “I’m Jane.   I live in France.   I like pizza.”
;; “I’m Jack.   I live in China.   I like chicken feet.”
;; ‘I’m Jim.  I live in China.”

;; It should be able to answer the following questions appropriately

;; “Does Jim like chicken feet?” -- “Yes” or “I think he does”
;; “Does Jim like pizza?” -- “No’ or “I don’t know”

;; Due to anaphora resolution not working well we express the above as
;; follows
(chat "Bob lives in China")
(chat "Bob likes chicken feet")

(chat "Jane lives in France")
(chat "Jane likes pizza")

;; (chat "What does Jane like?")

;; Janes likes pizza
;;
;; (SetLink
;;    (InheritanceLink
;;       (ConceptNode "Jane@5ab32607-6db5-4066-99eb-986afbf2a5a7")
;;       (ConceptNode "Jane" (stv 0.032258064 0.0012484394))
;;    )
;;    (EvaluationLink
;;       (DefinedLinguisticPredicateNode "definite")
;;       (ListLink
;;          (ConceptNode "Jane@5ab32607-6db5-4066-99eb-986afbf2a5a7")
;;       )
;;    )
;;    (ImplicationLink
;;       (PredicateNode "likes@a21229d1-107f-42f8-b15a-c388b9ec96f1")
;;       (PredicateNode "like" (stv 0.25 0.0012484394))
;;    )
;;    (InheritanceLink
;;       (ConceptNode "pizza@ab309111-0588-4e85-9370-a90c84426a6e")
;;       (ConceptNode "pizza" (stv 0.032258064 0.0012484394))
;;    )
;;    (EvaluationLink
;;       (PredicateNode "likes@a21229d1-107f-42f8-b15a-c388b9ec96f1")
;;       (ListLink
;;          (ConceptNode "Jane@5ab32607-6db5-4066-99eb-986afbf2a5a7")
;;          (ConceptNode "pizza@ab309111-0588-4e85-9370-a90c84426a6e")
;;       )
;;    )
;;    (EvaluationLink
;;       (PredicateNode "likes@a21229d1-107f-42f8-b15a-c388b9ec96f1")
;;       (ListLink
;;          (ConceptNode "Jane@5ab32607-6db5-4066-99eb-986afbf2a5a7")
;;       )
;;    )
;;    (InheritanceLink
;;       (InterpretationNode "sentence@6b4bb69b-e94f-43a6-b8ab-739abd53a758_parse_0_interpretation_$X")
;;       (DefinedLinguisticConceptNode "DeclarativeSpeechAct")
;;    )
;;    (InheritanceLink
;;       (SpecificEntityNode "Jane@5ab32607-6db5-4066-99eb-986afbf2a5a7")
;;       (DefinedLinguisticConceptNode "female" (stv 0.055555556 0.0012484394))
;;    )
;;    (InheritanceLink
;;       (SpecificEntityNode "Jane@5ab32607-6db5-4066-99eb-986afbf2a5a7")
;;       (ConceptNode "Jane" (stv 0.032258064 0.0012484394))
;;    )
;;    (InheritanceLink
;;       (PredicateNode "likes@a21229d1-107f-42f8-b15a-c388b9ec96f1")
;;       (DefinedLinguisticConceptNode "present")
;;    )
;; )

(chat "Jack lives in China")
(chat "Jack likes chicken feet")

(chat "Jim lives in China")
(chat "What does Jim like?")
