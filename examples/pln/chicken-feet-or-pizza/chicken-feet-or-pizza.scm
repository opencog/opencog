;; Work in progress! This demo is a mess and is not ready for the
;; lambda user!
;;
;; To have this work you need to disable the advmod rule in
;; opencog/nlp/relex2logic/loader/gen-r2l-en-rulebase.scm and
;; reinstall!
;;
;; Of course you need to have the opencog relex server running. The
;; easiest way to achieve that is with docker, running
;;
;; <DOCKER_ROOT>/opencog/relex/run-opencog-relex.sh
;;
;; obtained from https://github.com/opencog/docker

;; Inference example combining the openpsi chatbot and PLN
;;
;; Given the following facts (obtaine via NLP)
;;
;; “I’m Bob.   I live in China.   I like chicken feet.”
;; “I’m Jane.   I live in France.   I like pizza.”
;; “I’m Jack.   I live in China.   I like chicken feet.”
;; ‘I’m Jim.  I live in China.”
;;
;; It should be able to answer the following questions appropriately
;;
;; “Does Jim like chicken feet?” -- “Yes” or “I think he does”
;; “Does Jim like pizza?” -- “No’ or “I don’t know”

;; Load the chatbot
(add-to-load-path "../../../opencog/nlp/chatbot-psi")
(load-from-path "chatbot.scm")

;; Convenient fetchers
(use-modules (opencog query))
(define (get-parse-nodes)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$P")
                           (Type "ParseNode"))
                         (Variable "$P"))))

;; Get the r2l output
(define (get-set-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$S")
                           (Type "SetLink"))
                         (Variable "$S"))))

(define (get-wordinstance-nodes)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$W")
                           (Type "WordInstanceNode"))
                         (Variable "$W"))))

;; Get all of them
(define (get-wordinstance-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$W")
                           (Type "WordInstanceLink"))
                         (Variable "$W"))))

;; Get all of them
(define (get-wordsequence-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$W")
                           (Type "WordSequenceLink"))
                         (Variable "$W"))))

;; Get all of them
(define (get-lemma-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$L")
                           (Type "LemmaLink"))
                         (Variable "$L"))))

;; Get all of them
(define (get-reference-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$R")
                           (Type "ReferenceLink"))
                         (Variable "$R"))))

;; Get all of them
(define (get-interpretation-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$I")
                           (Type "InterpretationLink"))
                         (Variable "$I"))))

;; Get all of them
(define (get-execution-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$E")
                           (Type "ExecutionLink"))
                         (Variable "$E"))))

;;;;;;;;;;;;;;;;;;;;;
;; Sub experiments ;;
;;;;;;;;;;;;;;;;;;;;;

(chat "Robert likes pizza")

;; corresponding r2l structure
   (SetLink
      (InheritanceLink
         (ConceptNode "Robert@2291e65a-e782-45ed-a849-90eabb0d9c1b")
         (ConceptNode "Robert" (stv 0.028571429 0.0012484394))
      )
      (EvaluationLink
         (DefinedLinguisticPredicateNode "definite")
         (ListLink
            (ConceptNode "Robert@2291e65a-e782-45ed-a849-90eabb0d9c1b")
         )
      )
      (ImplicationLink
         (PredicateNode "likes@44a7a36f-07e5-48ec-bd51-7dd2277f3464")
         (PredicateNode "like" (stv 0.25 0.0012484394))
      )
      (InheritanceLink
         (ConceptNode "pizza@31c87ccb-0f52-4c05-8112-53c16cc294e3")
         (ConceptNode "pizza" (stv 0.028571429 0.0012484394))
      )
      (EvaluationLink
         (PredicateNode "likes@44a7a36f-07e5-48ec-bd51-7dd2277f3464")
         (ListLink
            (ConceptNode "Robert@2291e65a-e782-45ed-a849-90eabb0d9c1b")
            (ConceptNode "pizza@31c87ccb-0f52-4c05-8112-53c16cc294e3")
         )
      )
      (EvaluationLink
         (PredicateNode "likes@44a7a36f-07e5-48ec-bd51-7dd2277f3464")
         (ListLink
            (ConceptNode "Robert@2291e65a-e782-45ed-a849-90eabb0d9c1b")
         )
      )
      (InheritanceLink
         (InterpretationNode "sentence@f3a3ed98-afa1-4928-88d2-e2ce4bbb8a4e_parse_0_interpretation_$X")
         (DefinedLinguisticConceptNode "DeclarativeSpeechAct")
      )
      (InheritanceLink
         (PredicateNode "likes@44a7a36f-07e5-48ec-bd51-7dd2277f3464")
         (DefinedLinguisticConceptNode "present")
      )
   )
   (SetLink
      (ReferenceLink
         (SentenceNode "sentence@f3a3ed98-afa1-4928-88d2-e2ce4bbb8a4e")
         (Node "Robert likes pizza")
         (ListLink
            (WordNode "Robert" (stv 0.035714287 0.0012484394))
            (WordNode "likes" (stv 0.035714287 0.0012484394))
            (WordNode "pizza" (stv 0.035714287 0.0012484394))
         )
      )
   )

(chat "Alice likes pizza")

(chat "What does Robert like?")

;; Call sureal with "Robert likes pizza" structure
(Word "Robert") (sureal (Set (Evaluation (Predicate "likes") (List (Concept "Robert") (Concept "pizza")))))

;;;;;;;;;;;;;;;;;
;; Sub-exp end ;;
;;;;;;;;;;;;;;;;;


;; To get the r2l outputs of a parse use
;;
;; (parse-get-r2l-outputs parse)
;;
;; where parse can be gotten using
;;
;; (get-parse-nodes)

;; Due to anaphora resolution not working well we express
;;
;; “I’m Bob.   I live in China.   I like chicken feet.”
;;
;; as follows (also, Bob has been replaced by Robert because LG
;; doesn't like Bob).
(chat "Robert lives in China")
;; (chat "Robert likes chicken feet")

;; r2l corresponding to Robert lives in China

   (SetLink
      (InheritanceLink
         (ConceptNode "in@1a483359-e626-49d9-a52e-838196b07d4d")
         (ConceptNode "in" (stv 0.027027028 0.0012484394))
      )
      (ImplicationLink
         (PredicateNode "lives@59fae2d9-5d64-476b-962e-a706a9990905")
         (PredicateNode "live" (stv 0.16666667 0.0012484394))
      )
      (InheritanceLink
         (SatisfyingSetLink
            (PredicateNode "lives@59fae2d9-5d64-476b-962e-a706a9990905")
         )
         (ConceptNode "in@1a483359-e626-49d9-a52e-838196b07d4d")
      )
      (InheritanceLink
         (ConceptNode "China@d4b1ce02-5277-4734-96b3-3e95443d7136")
         (ConceptNode "China" (stv 0.027027028 0.0012484394))
      )
      (ImplicationLink
         (PredicateNode "in@1a483359-e626-49d9-a52e-838196b07d4d")
         (PredicateNode "in" (stv 0.16666667 0.0012484394))
      )
      (EvaluationLink
         (PredicateNode "in@1a483359-e626-49d9-a52e-838196b07d4d")
         (ListLink
            (PredicateNode "lives@59fae2d9-5d64-476b-962e-a706a9990905")
            (ConceptNode "China@d4b1ce02-5277-4734-96b3-3e95443d7136")
         )
      )
      (EvaluationLink
         (DefinedLinguisticPredicateNode "definite")
         (ListLink
            (ConceptNode "China@d4b1ce02-5277-4734-96b3-3e95443d7136")
         )
      )
      (InheritanceLink
         (ConceptNode "Robert@de5b4eb3-16c8-4648-8673-54ed676efb72")
         (ConceptNode "Robert" (stv 0.027027028 0.0012484394))
      )
      (EvaluationLink
         (DefinedLinguisticPredicateNode "definite")
         (ListLink
            (ConceptNode "Robert@de5b4eb3-16c8-4648-8673-54ed676efb72")
         )
      )
      (EvaluationLink
         (PredicateNode "in@1a483359-e626-49d9-a52e-838196b07d4d")
         (ListLink
            (ConceptNode "China@d4b1ce02-5277-4734-96b3-3e95443d7136")
         )
      )
      (EvaluationLink
         (PredicateNode "lives@59fae2d9-5d64-476b-962e-a706a9990905")
         (ListLink
            (ConceptNode "Robert@de5b4eb3-16c8-4648-8673-54ed676efb72")
         )
      )
      (InheritanceLink
         (InterpretationNode "sentence@d0996a37-27c4-451c-a9ec-33c385c9a498_parse_0_interpretation_$X")
         (DefinedLinguisticConceptNode "DeclarativeSpeechAct")
      )
      (InheritanceLink
         (SpecificEntityNode "China@d4b1ce02-5277-4734-96b3-3e95443d7136")
         (DefinedLinguisticConceptNode "female" (stv 0.055555556 0.0012484394))
      )
      (InheritanceLink
         (SpecificEntityNode "China@d4b1ce02-5277-4734-96b3-3e95443d7136")
         (ConceptNode "China" (stv 0.027027028 0.0012484394))
      )
      (InheritanceLink
         (PredicateNode "lives@59fae2d9-5d64-476b-962e-a706a9990905")
         (DefinedLinguisticConceptNode "present")
      )
   )



;; ;; Same thing for Jane
;; (chat "Jane lives in France")
;; (chat "Jane likes pizza")

;; ;; Facts about Jack (we replace by Joe because LG doesn't like Jack either)
;; (chat "Joe lives in China")
;; (chat "Joe likes chicken feet")

;; ;; Partial facts about Jim.
;; (chat "Jim lives in China")

;; ;; Now reasoning should be used to infer the answer of the following.
;; (chat "What does Jim like?")

;;
;; Inference experimentation
;;

;; r2l output corresponding to (chat "Bob lives in China")
;;
;; This is clearly wrong (because LG doesn't like Bob), but we keep
;; around just in case.
;;
;;    (SetLink
;;       (InheritanceLink
;;          (ConceptNode "in@b8dbff0f-ffe4-4775-9905-f9c87f29f63d")
;;          (ConceptNode "in" (stv 0.032258064 0.0012484394))
;;       )
;;       (ImplicationLink
;;          (PredicateNode "bob@04c3177d-bb4d-4a44-ae30-f4acbf1ea59f")
;;          (PredicateNode "bob" (stv 0.16666667 0.0012484394))
;;       )
;;       (InheritanceLink
;;          (SatisfyingSetLink
;;             (PredicateNode "bob@04c3177d-bb4d-4a44-ae30-f4acbf1ea59f")
;;          )
;;          (ConceptNode "in@b8dbff0f-ffe4-4775-9905-f9c87f29f63d")
;;       )
;;       (InheritanceLink
;;          (ConceptNode "China@b5d7ee9c-4cb1-4a70-b7ea-4dec2db35558")
;;          (ConceptNode "China" (stv 0.032258064 0.0012484394))
;;       )
;;       (ImplicationLink
;;          (PredicateNode "in@b8dbff0f-ffe4-4775-9905-f9c87f29f63d")
;;          (PredicateNode "in" (stv 0.16666667 0.0012484394))
;;       )
;;       (EvaluationLink
;;          (PredicateNode "in@b8dbff0f-ffe4-4775-9905-f9c87f29f63d")
;;          (ListLink
;;             (PredicateNode "bob@04c3177d-bb4d-4a44-ae30-f4acbf1ea59f")
;;             (ConceptNode "China@b5d7ee9c-4cb1-4a70-b7ea-4dec2db35558")
;;          )
;;       )
;;       (EvaluationLink
;;          (DefinedLinguisticPredicateNode "definite")
;;          (ListLink
;;             (ConceptNode "China@b5d7ee9c-4cb1-4a70-b7ea-4dec2db35558")
;;          )
;;       )
;;       (EvaluationLink
;;          (PredicateNode "in@b8dbff0f-ffe4-4775-9905-f9c87f29f63d")
;;          (ListLink
;;             (ConceptNode "China@b5d7ee9c-4cb1-4a70-b7ea-4dec2db35558")
;;          )
;;       )
;;       (InheritanceLink
;;          (InterpretationNode "sentence@0c2ab58f-7150-4259-8cd2-52ddf260d7f1_parse_0_interpretation_$X")
;;          (DefinedLinguisticConceptNode "ImperativeSpeechAct")
;;       )
;;       (InheritanceLink
;;          (SpecificEntityNode "China@b5d7ee9c-4cb1-4a70-b7ea-4dec2db35558")
;;          (DefinedLinguisticConceptNode "female" (stv 0.050000001 0.0012484394))
;;       )
;;       (InheritanceLink
;;          (SpecificEntityNode "China@b5d7ee9c-4cb1-4a70-b7ea-4dec2db35558")
;;          (ConceptNode "China" (stv 0.032258064 0.0012484394))
;;       )
;;       (InheritanceLink
;;          (PredicateNode "bob@04c3177d-bb4d-4a44-ae30-f4acbf1ea59f")
;;          (DefinedLinguisticConceptNode "imperative")
;;       )
;;    )

;; ;; r2l output corresponding to (chat "Bob likes chicken feet")

;;    (SetLink
;;       (InheritanceLink
;;          (ConceptNode "Bob@c0bce0fd-d30f-402e-8897-259fa248473d")
;;          (ConceptNode "Bob" (stv 0.025641026 0.0012484394))
;;       )
;;       (EvaluationLink
;;          (DefinedLinguisticPredicateNode "definite")
;;          (ListLink
;;             (ConceptNode "Bob@c0bce0fd-d30f-402e-8897-259fa248473d")
;;          )
;;       )
;;       (ImplicationLink
;;          (PredicateNode "likes@dc51995f-6e69-4d8e-91a7-0d5992dba6c6")
;;          (PredicateNode "like" (stv 0.125 0.0012484394))
;;       )
;;       (InheritanceLink
;;          (ConceptNode "feet@68a4daa2-1f53-4c0f-8443-806099311b49")
;;          (ConceptNode "foot" (stv 0.025641026 0.0012484394))
;;       )
;;       (EvaluationLink
;;          (PredicateNode "likes@dc51995f-6e69-4d8e-91a7-0d5992dba6c6")
;;          (ListLink
;;             (ConceptNode "Bob@c0bce0fd-d30f-402e-8897-259fa248473d")
;;             (ConceptNode "feet@68a4daa2-1f53-4c0f-8443-806099311b49")
;;          )
;;       )
;;       (EvaluationLink
;;          (PredicateNode "likes@dc51995f-6e69-4d8e-91a7-0d5992dba6c6")
;;          (ListLink
;;             (ConceptNode "Bob@c0bce0fd-d30f-402e-8897-259fa248473d")
;;          )
;;       )
;;       (InheritanceLink
;;          (InterpretationNode "sentence@e943fc5b-cf61-4ed1-a172-ab09d545ae67_parse_0_interpretation_$X")
;;          (DefinedLinguisticConceptNode "DeclarativeSpeechAct")
;;       )
;;       (InheritanceLink
;;          (ConceptNode "chicken@43d36c82-1eb0-4e42-b694-68e21023b5fa")
;;          (ConceptNode "chicken" (stv 0.025641026 0.0012484394))
;;       )
;;       (InheritanceLink
;;          (ConceptNode "feet@68a4daa2-1f53-4c0f-8443-806099311b49")
;;          (ConceptNode "chicken@43d36c82-1eb0-4e42-b694-68e21023b5fa")
;;       )
;;       (InheritanceLink
;;          (SpecificEntityNode "Bob@c0bce0fd-d30f-402e-8897-259fa248473d")
;;          (DefinedLinguisticConceptNode "male")
;;       )
;;       (InheritanceLink
;;          (SpecificEntityNode "Bob@c0bce0fd-d30f-402e-8897-259fa248473d")
;;          (ConceptNode "Bob" (stv 0.025641026 0.0012484394))
;;       )
;;       (InheritanceLink
;;          (PredicateNode "likes@dc51995f-6e69-4d8e-91a7-0d5992dba6c6")
;;          (DefinedLinguisticConceptNode "present")
;;       )
;;    )

;; ;; the following is what is generated by the chatbot when entering
;; ;; "Jane likes pizza". All (or almost all) of that turns out to be
;; ;; useful for NLG processing to generate the answer to the question
;; ;;
;; ;; (chat "What does Jane like?")

;;    ;; r2l output
;;    (SetLink
;;       (InheritanceLink
;;          (ConceptNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;          (ConceptNode "Jane" (stv 0.032258064 0.0012484394))
;;       )
;;       (EvaluationLink
;;          (DefinedLinguisticPredicateNode "definite")
;;          (ListLink
;;             (ConceptNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;          )
;;       )
;;       (ImplicationLink
;;          (PredicateNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;          (PredicateNode "like" (stv 0.25 0.0012484394))
;;       )
;;       (InheritanceLink
;;          (ConceptNode "pizza@0d996d7f-2522-45f0-95fd-7a3351caca27")
;;          (ConceptNode "pizza" (stv 0.032258064 0.0012484394))
;;       )
;;       (EvaluationLink
;;          (PredicateNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;          (ListLink
;;             (ConceptNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;             (ConceptNode "pizza@0d996d7f-2522-45f0-95fd-7a3351caca27")
;;          )
;;       )
;;       (EvaluationLink
;;          (PredicateNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;          (ListLink
;;             (ConceptNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;          )
;;       )
;;       (InheritanceLink
;;          (InterpretationNode "sentence@a88796c5-be60-4036-beb1-39bf193c1551_parse_0_interpretation_$X")
;;          (DefinedLinguisticConceptNode "DeclarativeSpeechAct")
;;       )
;;       (InheritanceLink
;;          (SpecificEntityNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;          (DefinedLinguisticConceptNode "female" (stv 0.055555556 0.0012484394))
;;       )
;;       (InheritanceLink
;;          (SpecificEntityNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;          (ConceptNode "Jane" (stv 0.032258064 0.0012484394))
;;       )
;;       (InheritanceLink
;;          (PredicateNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;          (DefinedLinguisticConceptNode "present")
;;       )
;;    )

;;    ;; Lemma links (required by nlp fuzzy matcher)
;;    (LemmaLink (stv 1 1)
;;       (WordInstanceNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;       (WordNode "Jane" (stv 0.034482758 0.0012484394))
;;    )
;;    (LemmaLink (stv 1 1)
;;       (WordInstanceNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;       (WordNode "like")
;;    )
;;    (LemmaLink (stv 1 1)
;;       (WordInstanceNode "pizza@0d996d7f-2522-45f0-95fd-7a3351caca27")
;;       (WordNode "pizza" (stv 0.034482758 0.0012484394))
;;    )

;;    ;; Word instance links (required by sureal)
;;    (WordInstanceLink (stv 1 1)
;;       (WordInstanceNode "LEFT-WALL@sentence@a88796c5-be60-4036-beb1-39bf193c1551_parse_0")
;;       (ParseNode "sentence@a88796c5-be60-4036-beb1-39bf193c1551_parse_0" (stv 1 0.98799998))
;;    )
;;    (WordInstanceLink (stv 1 1)
;;       (WordInstanceNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;       (ParseNode "sentence@a88796c5-be60-4036-beb1-39bf193c1551_parse_0" (stv 1 0.98799998))
;;    )
;;    (WordInstanceLink (stv 1 1)
;;       (WordInstanceNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;       (ParseNode "sentence@a88796c5-be60-4036-beb1-39bf193c1551_parse_0" (stv 1 0.98799998))
;;    )
;;    (WordInstanceLink (stv 1 1)
;;       (WordInstanceNode "pizza@0d996d7f-2522-45f0-95fd-7a3351caca27")
;;       (ParseNode "sentence@a88796c5-be60-4036-beb1-39bf193c1551_parse_0" (stv 1 0.98799998))
;;    )

;;    ;; Word sequence links (required by sureal)
;;    (WordSequenceLink (stv 1 1)
;;       (WordInstanceNode "LEFT-WALL@sentence@a88796c5-be60-4036-beb1-39bf193c1551_parse_0")
;;       (NumberNode "67.000000")
;;    )
;;    (WordSequenceLink (stv 1 1)
;;       (WordInstanceNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;       (NumberNode "68.000000")
;;    )
;;    (WordSequenceLink (stv 1 1)
;;       (WordInstanceNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;       (NumberNode "69.000000")
;;    )
;;    (WordSequenceLink (stv 1 1)
;;       (WordInstanceNode "pizza@0d996d7f-2522-45f0-95fd-7a3351caca27")
;;       (NumberNode "70.000000")
;;    )

;;    ;; Interpretation links (required by sureal)
;;    (InterpretationLink
;;       (InterpretationNode "sentence@a88796c5-be60-4036-beb1-39bf193c1551_parse_0_interpretation_$X")
;;       (ParseNode "sentence@a88796c5-be60-4036-beb1-39bf193c1551_parse_0" (stv 1 0.98799998))
;;    )

;;    ;; Reference links, not all are necessary (required by nlp fuzzy
;;    ;; matcher and sureal)
;;    (ReferenceLink
;;       (VariableNode "$wall-inst")
;;       (WordNode "###LEFT-WALL###" (stv 0.034482758 0.0012484394))
;;    )
;;    (ReferenceLink
;;       (VariableNode "$wall-inst")
;;       (WordNode ",")
;;    )
;;    (ReferenceLink
;;       (VariableNode "$l")
;;       (VariableNode "$x")
;;    )
;;    (ReferenceLink (stv 1 1)
;;       (WordInstanceNode "LEFT-WALL@sentence@a88796c5-be60-4036-beb1-39bf193c1551_parse_0")
;;       (WordNode "###LEFT-WALL###" (stv 0.034482758 0.0012484394))
;;    )
;;    (ReferenceLink (stv 1 1)
;;       (WordInstanceNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;       (WordNode "Jane" (stv 0.034482758 0.0012484394))
;;    )
;;    (ReferenceLink (stv 1 1)
;;       (WordInstanceNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;       (WordNode "likes" (stv 0.034482758 0.0012484394))
;;    )
;;    (ReferenceLink (stv 1 1)
;;       (WordInstanceNode "pizza@0d996d7f-2522-45f0-95fd-7a3351caca27")
;;       (WordNode "pizza" (stv 0.034482758 0.0012484394))
;;    )
;;    (ReferenceLink
;;       (LgLinkInstanceNode "Ss@fa647385-5321-40e3-8584-9e72241bc40c")
;;       (LinkGrammarRelationshipNode "Ss")
;;    )
;;    (ReferenceLink
;;       (LgLinkInstanceNode "Ou@00a316a2-3ec7-4fd1-87b5-ea405dee4f19")
;;       (LinkGrammarRelationshipNode "Ou")
;;    )
;;    (ReferenceLink
;;       (LgLinkInstanceNode "Wd@3c7099c0-add9-4632-aa30-e2c04601aeac")
;;       (LinkGrammarRelationshipNode "Wd")
;;    )
;;    (ReferenceLink
;;       (LgLinkInstanceNode "WV@2ef19b9e-b6a9-4589-b104-a7f8e313a2be")
;;       (LinkGrammarRelationshipNode "WV")
;;    )
;;    (ReferenceLink
;;       (ConceptNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;       (WordInstanceNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;    )
;;    (ReferenceLink
;;       (PredicateNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;       (WordInstanceNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;    )
;;    (ReferenceLink
;;       (ConceptNode "pizza@0d996d7f-2522-45f0-95fd-7a3351caca27")
;;       (WordInstanceNode "pizza@0d996d7f-2522-45f0-95fd-7a3351caca27")
;;    )
;;    (ReferenceLink
;;       (InterpretationNode "sentence@a88796c5-be60-4036-beb1-39bf193c1551_parse_0_interpretation_$X")
;;       (SetLink
;;          (InheritanceLink
;;             (ConceptNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;             (ConceptNode "Jane" (stv 0.032258064 0.0012484394))
;;          )
;;          (EvaluationLink
;;             (DefinedLinguisticPredicateNode "definite")
;;             (ListLink
;;                (ConceptNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;             )
;;          )
;;          (ImplicationLink
;;             (PredicateNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;             (PredicateNode "like" (stv 0.25 0.0012484394))
;;          )
;;          (InheritanceLink
;;             (ConceptNode "pizza@0d996d7f-2522-45f0-95fd-7a3351caca27")
;;             (ConceptNode "pizza" (stv 0.032258064 0.0012484394))
;;          )
;;          (EvaluationLink
;;             (PredicateNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;             (ListLink
;;                (ConceptNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;                (ConceptNode "pizza@0d996d7f-2522-45f0-95fd-7a3351caca27")
;;             )
;;          )
;;          (EvaluationLink
;;             (PredicateNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;             (ListLink
;;                (ConceptNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;             )
;;          )
;;          (InheritanceLink
;;             (InterpretationNode "sentence@a88796c5-be60-4036-beb1-39bf193c1551_parse_0_interpretation_$X")
;;             (DefinedLinguisticConceptNode "DeclarativeSpeechAct")
;;          )
;;          (InheritanceLink
;;             (SpecificEntityNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;             (DefinedLinguisticConceptNode "female" (stv 0.055555556 0.0012484394))
;;          )
;;          (InheritanceLink
;;             (SpecificEntityNode "Jane@e0ffaf00-8086-4682-b7ea-f318f9968516")
;;             (ConceptNode "Jane" (stv 0.032258064 0.0012484394))
;;          )
;;          (InheritanceLink
;;             (PredicateNode "likes@be81f834-60f1-4d91-8784-492353112606")
;;             (DefinedLinguisticConceptNode "present")
;;          )
;;       )
;;    )
;;    (ReferenceLink
;;       (ListLink
;;          (WordNode "Jane" (stv 0.034482758 0.0012484394))
;;          (WordNode "likes" (stv 0.034482758 0.0012484394))
;;          (WordNode "pizza" (stv 0.034482758 0.0012484394))
;;       )
;;       (SentenceNode "sentence@a88796c5-be60-4036-beb1-39bf193c1551")
;;    )

;; Halt openpsi to not be bothered with too much logging and
;; background process
(psi-halt)

;; Load PLN configuration for that demo.
(load "pln-config.scm")

;; Try to manually craft a rule telling that people living in China
;; like chicken feet. For now the rule has a full strength and
;; confidence.

;; (ImplicationLink (stv 1 1)
;;    ;;;;;;;;;;;;;;;;;;;;;;;;;;
;;    ;; Variable declaration ;;
;;    ;;;;;;;;;;;;;;;;;;;;;;;;;;
;;    (VariableList
;;       ;; Individual
;;       (TypedVariable
;;          (Variable "$individual")
;;          (Type "ConceptNode"))
;;       (TypedVariable
;;          (Variable "$individual-instance")
;;          (Type "ConceptNode"))
;;       ;; Live
;;       (TypedVariable
;;          (Variable "$live-instance")
;;          (Type "PredicateNode"))
;;       ;; In
;;       (TypedVariable
;;          (Variable "$in-instance")
;;          (Type "PredicateNode"))
;;       ;; China
;;       (TypedVariable
;;          (Variable "$China-instance")
;;          (Type "ConceptNode"))
;;       (TypedVariable
;;          (Variable "$China-specific-entity-instance")
;;          (Type "SpecificEntityNode"))
;;       ;; Interpretation
;;       (TypedVariable
;;          (Variable "$interpretation-instance")
;;          (Type "InterpretationNode"))
;;       ;; Like
;;       (TypedVariable
;;          (Variable "$like-instance")
;;          (Type "PredicateNode"))
;;       ;; Chicken feet
;;       (TypedVariable
;;          (Variable "$chicken-instance")
;;          (Type "ConceptNode"))
;;       (TypedVariable
;;          (Variable "$foot-instance")
;;          (Type "ConceptNode")))
;;    ;;;;;;;;;;;;;;;
;;    ;; Implicant ;;
;;    ;;;;;;;;;;;;;;;
;;    (And
;;       ;; Individual
;;       (Inheritance
;;          (Variable "$individual-instance")
;;          (Variable "$individual"))
;;       (EvaluationLink
;;          (DefinedLinguisticPredicateNode "definite")
;;          (ListLink
;;             (Variable "$individual-instance")))
;;       ;; Live
;;       (ImplicationLink
;;          (Variable "$live-instance")
;;          (PredicateNode "live"))
;;       (EvaluationLink
;;          (Variable "$live-instance")
;;          (ListLink
;;             (Variable "$individual-instance")))
;;       (InheritanceLink
;;          (Variable "$live-instance")
;;          (DefinedLinguisticConceptNode "present"))
;;       ;; In
;;       (ImplicationLink
;;          (Variable "$in-instance")
;;          (PredicateNode "in"))
;;       (EvaluationLink
;;          (Variable "$in-instance")
;;          (ListLink
;;             (Variable "$live-instance")
;;             (Variable "$China-instance")))
;;       (EvaluationLink
;;          (Variable "$in-instance")
;;          (ListLink
;;             (Variable "$China-instance")))
;;       ;; China
;;       (Inheritance
;;          (Variable "$China-instance")
;;          (Concept "China"))
;;       (EvaluationLink
;;          (DefinedLinguisticPredicateNode "definite")
;;          (ListLink
;;             (Variable "$China-instance")))
;;       (InheritanceLink
;;          (Variable "$China-specific-entity-instance")
;;          (DefinedLinguisticConceptNode "female"))
;;       (InheritanceLink
;;          (Variable "China-specific-entity-instance")
;;          (ConceptNode "China"))
;;       ;; Interpretation
;;       (InheritanceLink
;;          (Variable "$interpretation-instance")
;;          (DefinedLinguisticConceptNode "DeclarativeSpeechAct")))
;;    ;;;;;;;;;;;;;;;
;;    ;; Implicand ;;
;;    ;;;;;;;;;;;;;;;
;;    (Set
;;       ;; Individual
;;       (InheritanceLink
;;          (Variable "$individual-instance")
;;          (Variable "$individual"))
;;       (EvaluationLink
;;          (DefinedLinguisticPredicateNode "definite")
;;          (ListLink
;;             (Variable "$individual-instance")))
;;       ;; Like
;;       (ImplicationLink
;;          (Variable "$like-instance")
;;          (PredicateNode "like"))
;;       (EvaluationLink
;;          (Variable "$like-instance")
;;          (ListLink
;;             (Variable "$individual-instance")
;;             (Variable "$foot-instance")))
;;       (EvaluationLink
;;          (Variable "$like-instance")
;;          (ListLink
;;             (ConceptNode "$individual-instance")))
;;       (InheritanceLink
;;          (Variable "$like-instance")
;;          (DefinedLinguisticConceptNode "present"))
;;       ;; Chicken feet
;;       (InheritanceLink
;;          (Variable "$foot-instance")
;;          (ConceptNode "foot"))
;;       (InheritanceLink
;;          (Variable "$chicken-instance")
;;          (ConceptNode "chicken"))
;;       (InheritanceLink
;;          (VariableNode "$foot-instance")
;;          (ConceptNode "chicken-instance"))))

(ImplicationLink (stv 1 1)
   ;;;;;;;;;;;;;;;;;;;;;;;;;;
   ;; Variable declaration ;;
   ;;;;;;;;;;;;;;;;;;;;;;;;;;
   (VariableList
      ;; Individual
      (TypedVariable
         (Variable "$individual")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$individual-instance")
         (Type "ConceptNode"))
      ;; Live
      (TypedVariable
         (Variable "$live-instance")
         (Type "PredicateNode"))
      ;; In
      (TypedVariable
         (Variable "$in-instance")
         (Type "PredicateNode"))
      ;; China
      (TypedVariable
         (Variable "$China-instance")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$China-specific-entity-instance")
         (Type "SpecificEntityNode"))
      ;; Interpretation
      (TypedVariable
         (Variable "$interpretation-instance")
         (Type "InterpretationNode"))
      ;; Like
      (TypedVariable
         (Variable "$like-instance")
         (Type "PredicateNode"))
      ;; Chicken feet
      (TypedVariable
         (Variable "$chicken-instance")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$foot-instance")
         (Type "ConceptNode")))
   ;;;;;;;;;;;;;;;
   ;; implicant ;;
   ;;;;;;;;;;;;;;;
   (And
      ;; Individual
      (Inheritance
         (Variable "$individual-instance")
         (Variable "$individual"))
      (EvaluationLink
         (DefinedLinguisticPredicateNode "definite")
         (ListLink
            (Variable "$individual-instance")))
      ;; Live
      (ImplicationLink
         (Variable "$live-instance")
         (PredicateNode "live"))
      (EvaluationLink
         (Variable "$live-instance")
         (ListLink
            (Variable "$individual-instance")))
      (InheritanceLink
         (Variable "$live-instance")
         (DefinedLinguisticConceptNode "present"))
      ;; In
      (ImplicationLink
         (Variable "$in-instance")
         (PredicateNode "in"))
      (EvaluationLink
         (Variable "$in-instance")
         (ListLink
            (Variable "$live-instance")
            (Variable "$China-instance")))
      (EvaluationLink
         (Variable "$in-instance")
         (ListLink
            (Variable "$China-instance")))
      ;; China
      (Inheritance
         (Variable "$China-instance")
         (Concept "China"))
      (EvaluationLink
         (DefinedLinguisticPredicateNode "definite")
         (ListLink
            (Variable "$China-instance")))
      (InheritanceLink
         (Variable "$China-specific-entity-instance")
         (DefinedLinguisticConceptNode "female"))
      (InheritanceLink
         (Variable "China-specific-entity-instance")
         (ConceptNode "China"))
      ;; Interpretation
      (InheritanceLink
         (Variable "$interpretation-instance")
         (DefinedLinguisticConceptNode "DeclarativeSpeechAct")))
   ;;;;;;;;;;;;;;;
   ;; Implicand ;;
   ;;;;;;;;;;;;;;;
   (Set
      ;; Individual
      (InheritanceLink
         (Variable "$individual-instance")
         (Variable "$individual"))
      (EvaluationLink
         (DefinedLinguisticPredicateNode "definite")
         (ListLink
            (Variable "$individual-instance")))
      ;; Like
      (ImplicationLink
         (Variable "$like-instance")
         (PredicateNode "like"))
      (EvaluationLink
         (Variable "$like-instance")
         (ListLink
            (Variable "$individual-instance")
            (Variable "$foot-instance")))
      (EvaluationLink
         (Variable "$like-instance")
         (ListLink
            (VariableNode "$individual-instance")))
      (InheritanceLink
         (Variable "$like-instance")
         (DefinedLinguisticConceptNode "present"))
      ;; Chicken feet
      (InheritanceLink
         (Variable "$foot-instance")
         (ConceptNode "foot"))
      (InheritanceLink
         (Variable "$chicken-instance")
         (ConceptNode "chicken"))
      (InheritanceLink
         (VariableNode "$foot-instance")
         (VariableNode "$chicken-instance"))))

;; Try to instantiate that with (chat "Robert lives in China")

   ;; (SetLink
   ;;    ;; Individual
   ;;    (InheritanceLink
   ;;       (ConceptNode "Robert@aa3b573a-24f0-49c6-8441-1d98284ed7b1")
   ;;       (ConceptNode "Robert" (stv 0.032258064 0.0012484394))
   ;;    )
   ;;    (EvaluationLink
   ;;       (DefinedLinguisticPredicateNode "definite")
   ;;       (ListLink
   ;;          (ConceptNode "Robert@aa3b573a-24f0-49c6-8441-1d98284ed7b1")
   ;;       )
   ;;    )
   ;;    ;; Live
   ;;    (ImplicationLink
   ;;       (PredicateNode "lives@ca40392c-ea28-4968-b87c-256959e99b67")
   ;;       (PredicateNode "live" (stv 0.16666667 0.0012484394))
   ;;    )
   ;;    (EvaluationLink
   ;;       (PredicateNode "lives@ca40392c-ea28-4968-b87c-256959e99b67")
   ;;       (ListLink
   ;;          (ConceptNode "Robert@aa3b573a-24f0-49c6-8441-1d98284ed7b1")
   ;;       )
   ;;    )
   ;;    (InheritanceLink
   ;;       (PredicateNode "lives@ca40392c-ea28-4968-b87c-256959e99b67")
   ;;       (DefinedLinguisticConceptNode "present")
   ;;    )
   ;;    ;; In
   ;;    (ImplicationLink
   ;;       (PredicateNode "in@6fdda22e-445f-45ac-9707-8e2d626f33ae")
   ;;       (PredicateNode "in" (stv 0.16666667 0.0012484394))
   ;;    )
   ;;    (EvaluationLink
   ;;       (PredicateNode "in@6fdda22e-445f-45ac-9707-8e2d626f33ae")
   ;;       (ListLink
   ;;          (PredicateNode "lives@ca40392c-ea28-4968-b87c-256959e99b67")
   ;;          (ConceptNode "China@43d35662-4c04-4a32-b05c-d39f21acd2eb")
   ;;       )
   ;;    )
   ;;    (EvaluationLink
   ;;       (PredicateNode "in@6fdda22e-445f-45ac-9707-8e2d626f33ae")
   ;;       (ListLink
   ;;          (ConceptNode "China@43d35662-4c04-4a32-b05c-d39f21acd2eb")
   ;;       )
   ;;    )
   ;;    ;; China
   ;;    (InheritanceLink
   ;;       (ConceptNode "China@43d35662-4c04-4a32-b05c-d39f21acd2eb")
   ;;       (ConceptNode "China" (stv 0.032258064 0.0012484394))
   ;;    )
   ;;    (EvaluationLink
   ;;       (DefinedLinguisticPredicateNode "definite")
   ;;       (ListLink
   ;;          (ConceptNode "China@43d35662-4c04-4a32-b05c-d39f21acd2eb")
   ;;       )
   ;;    )
   ;;    (InheritanceLink
   ;;       (SpecificEntityNode "China@43d35662-4c04-4a32-b05c-d39f21acd2eb")
   ;;       (DefinedLinguisticConceptNode "female" (stv 0.055555556 0.0012484394))
   ;;    )
   ;;    (InheritanceLink
   ;;       (SpecificEntityNode "China@43d35662-4c04-4a32-b05c-d39f21acd2eb")
   ;;       (ConceptNode "China" (stv 0.032258064 0.0012484394))
   ;;    )
   ;;    ;; Interpretation
   ;;    (InheritanceLink
   ;;       (InterpretationNode "sentence@439f14be-5f35-47f1-8b0a-e503cabe05c0_parse_0_interpretation_$X")
   ;;       (DefinedLinguisticConceptNode "DeclarativeSpeechAct")
   ;;    )
   ;; )

;; Enable logging
(use-modules (opencog logger))
;; (cog-logger-set-level! "fine")
;; (cog-logger-set-stdout! #t)

;; Apply conditional instantiation [on that implication]
(cog-bind implication-full-instantiation-rule)
