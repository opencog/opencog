; simple-training.scm
;
; Experimental, simple training for hanson bots--on-the-fly creation of new
; behavior trues through natural language interaction with the robot.
;
; Starting off with simple, rigid command template to learn new stimulus and
; associated behavior.
; Example:
; User says to robot, "When I say 'Eddie is a stud' then you laugh."

(use-modules (opencog query))

; load kino's atomese string match code
(load "../aiml2oc/aiml2oc_guile/code/OpenCogAimlReply1.scm")

(clear)

;-----------------------------------------------------------------
; Behavior Trees
; TODO: these should go in a rules or behavior-trees file

(BindLink
    (ListLink
        (ConceptNode "YOU")
        (ConceptNode "ARE")
        (ConceptNode "BEAUTIFUL")
    )
    (EvaluationLink
  	    (PredicateNode "express-action")
      	(ListLink
  	    	(ConceptNode "pred-express")
  		    (DefinedSchema "happy")))

    ; or should it be:
    ;(EvaluationLink
    ;      (DefinedPredicateNode "Do show expression")
    ;      (ListLink
    ;         (ConceptNode "imperative")
    ;        (ConceptNode "happy" (ptv 1 0 1))
    ;     )
    ;   )

)

(BindLink
  (ListLink
    (ConceptNode "ARE")
    (ConceptNode "YOU")
    (ConceptNode "BORED")
  )
  (EvaluationLink
  	(PredicateNode "express-action")
  	(ListLink
  		(ConceptNode "pred-express")
  		(DefinedSchema "yawn-1")))
)


;-----------------------------------------------------------------
; AIML-style string matching to behavior condition

(define (get-tree-with-condition stimulus)
    (findQueryPatterns (cleanText (string-upcase (string-trim-both stimulus))))
)

;-----------------------------------------------------------------
; Utils
; TODO: move to util file



;-----------------------------------------------------------------
(define (create-behavior-tree stimulus response)
    ; this is where the new behavior tree will be dynamically create - coming
    ; back to this after getting the execution of AIML-style string matching
    ; of imperative stimulus working
    (display)
)

