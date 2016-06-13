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

;; Jane likes pizza
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

;; What does Jane like?
;;
;; (SetLink
;;    (InheritanceLink
;;       (ConceptNode "Jane@2fd0dd6e-a31f-4b61-963a-0f2d6e3b8cdb")
;;       (ConceptNode "Jane" (stv 0.032258064 0.0012484394))
;;    )
;;    (EvaluationLink
;;       (DefinedLinguisticPredicateNode "definite")
;;       (ListLink
;;          (ConceptNode "Jane@2fd0dd6e-a31f-4b61-963a-0f2d6e3b8cdb")
;;       )
;;    )
;;    (ImplicationLink
;;       (PredicateNode "like@58211994-e4a6-41fb-aa1b-ef0b999277a7")
;;       (PredicateNode "like" (stv 0.25 0.0012484394))
;;    )
;;    (InheritanceLink
;;       (ConceptNode "what@c7fe43d5-7899-47fb-a2a2-9e51bd32dc03")
;;       (ConceptNode "what" (stv 0.032258064 0.0012484394))
;;    )
;;    (EvaluationLink
;;       (PredicateNode "like@58211994-e4a6-41fb-aa1b-ef0b999277a7")
;;       (ListLink
;;          (ConceptNode "Jane@2fd0dd6e-a31f-4b61-963a-0f2d6e3b8cdb")
;;          (ConceptNode "what@c7fe43d5-7899-47fb-a2a2-9e51bd32dc03")
;;       )
;;    )
;;    (EvaluationLink
;;       (PredicateNode "like@58211994-e4a6-41fb-aa1b-ef0b999277a7")
;;       (ListLink
;;          (ConceptNode "Jane@2fd0dd6e-a31f-4b61-963a-0f2d6e3b8cdb")
;;       )
;;    )
;;    (InheritanceLink
;;       (InterpretationNode "sentence@67c4bd87-d367-42f7-9427-758e6f9bccf4_parse_0_interpretation_$X")
;;       (DefinedLinguisticConceptNode "InterrogativeSpeechAct")
;;    )
;;    (InheritanceLink
;;       (SpecificEntityNode "Jane@2fd0dd6e-a31f-4b61-963a-0f2d6e3b8cdb")
;;       (DefinedLinguisticConceptNode "female" (stv 0.045454547 0.0012484394))
;;    )
;;    (InheritanceLink
;;       (SpecificEntityNode "Jane@2fd0dd6e-a31f-4b61-963a-0f2d6e3b8cdb")
;;       (ConceptNode "Jane" (stv 0.032258064 0.0012484394))
;;    )
;;    (InheritanceLink
;;       (PredicateNode "like@58211994-e4a6-41fb-aa1b-ef0b999277a7")
;;       (DefinedLinguisticConceptNode "present_infinitive")
;;    )
;; )

(chat "Jack lives in China")
(chat "Jack likes chicken feet")

(chat "Jim lives in China")
(chat "What does Jim like?")
