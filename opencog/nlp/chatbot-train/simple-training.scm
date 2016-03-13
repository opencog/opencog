; simple-training.scm
;
; Experimental, simple training for hanson bots--on-the-fly creation of new
; behavior rules through natural language interaction with the robot.
;
; Starting off with simple, rigid command template to learn new stimulus and
; associated response behavior.
; Example:
; User says to robot, "When I say 'Eddie is a stud' then you laugh."
;
; Requires installation of opencog/ros-behavior-scripting

; TODO: probably can take out eva-behavior when express.scm is moved out
(use-modules (opencog) (opencog query) (opencog exec) (opencog eva-model)
    (opencog eva-behavior))

; load kino's atomese string match code
(load "../aiml2oc/aiml2oc_guile/code/OpenCogAimlReply1.scm")

; load config file for chatbot (eva)
(load-from-path "opencog/eva-behavior/cfg-eva.scm")

;-----------------------------------------------------------------
; The main training rule template for natural language training 
(define training-rule
;(DefineLink
;    (DefinedType "training rule")
    (BindLink
        (ListLink
            (ConceptNode "WHEN")
            (ConceptNode "I")
            (ConceptNode "SAY")
            (GlobNode "$stimulus")
            (ConceptNode "THEN")
            (ConceptNode "YOU")
            (GlobNode "$response")
        )
        (Evaluation
            (GroundedPredicateNode "scm:create-behavior-rule")
            (ListLink
                (GlobNode "$stimulus")
                (GlobNode "$response")))))



;-----------------------------------------------------------------
; Define behavior actions
; This should go in a behavior-actions.scm file

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
        (DefinedPredicateNode "Do show gesture")
        (ListLink
            (ConceptNode "imperative")
            (ConceptNode "yawn-1"))))




;-----------------------------------------------------------------
; Behavior Trees
; TODO: these should go in a behavior-rules.scm file

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


; TODO: below not working with multiple words in the glob

; Executes a behavior tree based on a stimulus input string.
; TODO; implment passing the grounded variables in the rule when evaluating it
(define (execute-behavior-with-stimulus input-str)
    (define rule (get-tree-with-antecedent input-str))
    (if rule
        (begin
                (display "doing cog-eval on:\n") (display (gdr rule))
                ;(cog-evaluate! (gdr rule))
                (cog-evaluate! (gar (cog-bind rule)))
        )
        #f
    )
)


        ;(if (equal? rule training-rule)
            ; This is a hack until implementing version that passes the
            ; the grounded var values to all behavior rules when evaluating
            ;(create-new-brule-hack input-str)
            ;(begin

                ;(cog-execute! rule)
            ;)

; shortcut method
(define (say input-str)
    (execute-behavior-with-stimulus input-str))


(define (string-to-atomese input)
    (define atomese-string)
    (define stim-and-response)
    (define cleaned-text
        (string-trim-both (cleanText (string-upcase input))))
    (set! atomese-string (genQueryPattern cleaned-text))
    (display "atomese-string: ")(atomese-string)(newline)

)



;-----------------------------------------------------------------
(define (create-behavior-rule stimulus response)
    ; create new behavior rule with text input stimulus and behavior response
    (define new-rule)
    (define atomese-string)
    (display "\ncreate-behavior-tree \n    stimulus: ")(display stimulus)
        (display "    response: ")(display response)(newline)
    ;(set! atomese-string (string-to-atomese stimulus))
	(set! new-rule
    	(BindLink
    	    (ListLink
    			stimulus
    	    )
			(DefinedPredicateNode  (string-downcase (cog-name response)))
		)
	)
	(display new-rule)
    (stv 1 1)
)


;-----------------------------------------------------------------
; Utils
; TODO: move to util file





;-----------------------------------------------------------------
; NOTES
; Why not use cog-bind to get the behavior tree rather than cog-recognize?






; This is a hack until implementing version of execute-behavior-with-stimulus
; that passes the the grounded var values to all behavior rules when evaluating
(define (create-new-brule-hack input-str)
    (define atomese-string)
    (define stim-and-response)
    (define cleaned-text
        (string-trim-both (cleanText (string-upcase input-str))))
    (cog-push-atomspace)
    ; put the query string into the temp atomspace as word list
    (set! atomese-string (genQueryPattern cleaned-text))
    (display "atomese-string: ")(atomese-string)(newline)
    ;(set! stim-and-response
    ;    (cog-execute!
    ;        (GetLink
    ;            (gdr training-rule))))
    ;(display "stim-and-response: ")(display stim-and-response)

    (cog-bind training-rule)

    (cog-pop-atomspace)
)


; TODO: ask linas about these
; maybe the DefinedType atom is being replaced in the cog-satisfy
; TODO: let's try cog-get-partner instead
; Check if an atom is the training rule
(define (training-rule? atom)
    (cog-satisfy
        (SatisfactionLink
            (DefineLink
                (DefinedType "training-rule")
                atom))))


(define (training-rule2? atom)
    (cog-get
        (GetLink
            (DefineLink
                (DefinedType "training-rule")
                atom))))



