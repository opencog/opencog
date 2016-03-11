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

(define (create-behavior-tree stimulus response)
    ; this is where the new behavior tree will be dynamically create - coming
    ; back to this after getting the execution of AIML-style string matching
    ; of imperative stimulus working
)

;-----------------------------------------------------------------
; Behavior Trees

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


(EvaluationLink
	(PredicateNode "express-action")
	(ListLink
		(ConceptNode "pred-express")
		(ConceptNode "schema-express")))