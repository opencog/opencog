; simple-training.scm
;
; Experimental, simple training for hanson bots--on-the-fly creation of new
; behavior trues through natural language interaction with the robot.
;
; Starting off with simple, rigid command template to learn new stimulus and
; associated behavior.
; Example:
; User says to robot, "When I say 'Eddie is a stud' then you laugh."

; TODO: probably can take out eva-behavior when express.scm is moved out
(use-modules (opencog) (opencog query) (opencog exec) (opencog eva-model)
    (opencog eva-behavior))

; load kino's atomese string match code
(load "../aiml2oc/aiml2oc_guile/code/OpenCogAimlReply1.scm")

; load config file for eva
(load-from-path "opencog/eva-behavior/cfg-eva.scm")

;-----------------------------------------------------------------
; The main training tree
(BindLink
    (ListLink
        (ConceptNode "WHEN")
        (ConceptNode "I")
        (ConceptNode "SAY")
        (GlobNode "$stimulus")
        ;(ConceptNode "THEN")
        ;(ConceptNode "YOU")
        ;(GlobNode "$response")
    )
    (Evaluation
  	    (GroundedPredicateNode "scm:create-behavior-tree")
      	(ListLink
            (GlobNode "$stimulus")
            (GlobNode "$response"))))



;-----------------------------------------------------------------
; Define behavior actions

; happy
(DefineLink
    (DefinedPredicateNode "be happy")
    (EvaluationLink
        (DefinedPredicateNode "Do show expression")
        (ListLink
          (ConceptNode "imperative")
          (ConceptNode "happy"))))
; yawn
(DefineLink
    (DefinedPredicateNode "yawn")
    (EvaluationLink
        (PredicateNode "express-action")
        (ListLink
            (ConceptNode "pred-express")
            (DefinedSchema "yawn-1"))))




;-----------------------------------------------------------------
; Behavior Trees
; TODO: these should go in a rules or behavior-trees file

(BindLink
    (ListLink
        (ConceptNode "YOU")
        (ConceptNode "ARE")
        (ConceptNode "BEAUTIFUL")
    )
    (DefinedPredicateNode "be happy")
)

(BindLink
    (ListLink
        (ConceptNode "YOU")
        (ConceptNode "ARE")
        (GlobNode "$blah")
        (ConceptNode "BEAUTIFUL")
    )
    (DefinedPredicateNode "be happy"))


(BindLink
  (ListLink
    (ConceptNode "ARE")
    (ConceptNode "YOU")
    (ConceptNode "BORED")
  )
  (DefinedPredicateNode "yawn")
)


;-----------------------------------------------------------------
; Execute behavior based on string stimulus using AIML-style string matching

; Retrieve atomspace behavior tree(s) with antecedent that contains atomese
; representation of the input string.
(define (get-tree-with-antecedent input-str)
    ; TODO: For now just using a single result, but we should handle multiple
    ;       returned results.
    ; TODO: Allow for variable words

    (let* ((cleaned-text
                (string-trim-both (cleanText (string-upcase input-str))))
           (results (findQueryPatterns cleaned-text)))
        (display "results:\n") (display results)
        (if (> (length (cog-outgoing-set results)) 0)
	        (gar results)
	        #f
	    )
    )
)


; Executes a behavior tree based on a stimulus input string.
(define (execute-behavior-with-stimulus input-str)
    (define btree (get-tree-with-antecedent input-str))
    (if btree
        (begin
            (display "doing cog-eval on:\n") (display (gdr btree))
            (cog-evaluate! (gdr btree)))
        #f
    )
)

;-----------------------------------------------------------------
; Utils
; TODO: move to util file



;-----------------------------------------------------------------
(define (create-behavior-tree input-str stimulus response)
    ; create new behavior rule with text input stimulus and behavior response
    (display "create-behavior-tree ")(display stimulus)(display " ")
        (display response)(newline)

    (stv 1 1)
)

;-----------------------------------------------------------------
; NOTES
; Why not use cog-bind to get the behavior tree rather than cog-recognize?
