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

(load "behavior-defs.scm")
(load "behavior-rules.scm")

; TODO: multiple globnode values are not matching
; TODO: are "leftover" listified strings being left in the atomspace?


;-----------------------------------------------------------------
; The main training rule template for natural language training 
(define training-rule
;    (DefinedType "training rule")
    (BindLink
        (ListLink
            (ConceptNode "WHEN")
            (ConceptNode "I")
            (ConceptNode "SAY")
            (GlobNode "$stimulus")
            ;(ConceptNode "THEN")
            (ConceptNode "YOU")
            (GlobNode "$response")
        )
        (Evaluation
            (GroundedPredicateNode "scm:create-behavior-rule")
            (ListLink
                (List (GlobNode "$stimulus"))
                (List (GlobNode "$response"))))))







;-----------------------------------------------------------------
; Execution of behaviors based on string stimulus using AIML-style string
; matching.

; TODO: below not working with multiple words in the glob

; This is the main function that controls execution of a behavior rule based on
; a stimulus input string, if there is a match with a rule conditional.
; TODO; implment passing the grounded variables in the rule when evaluating it
(define (execute-behavior-with-stimulus input-str)
    (define rule)
    (define bind-results)
    (define eval-results)
    (define as-orig)
    (define temp-rule)
    (define consequent)
	;(cog-push-atomspace)
	(define cleaned-text (clean-text input-str))
    ;(set! rule (get-tree-with-antecedent input-str))
    (set! rule (get-tree-with-antecedent cleaned-text))
    (if rule
        (begin
            (display "rule: ")(display rule)(newline)
            ;(display "doing cog-eval on:\n") (display (gdr rule))
            ;(cog-evaluate! (gdr rule))

            ; Need to figure out here how to unify the variables in the rule
            ; without binding the previous phrase matches that have happened
            ; Create and set a temp atomspace
            (set! as-orig (cog-set-atomspace! (cog-new-atomspace)))
            ; Add the rule and the listified input to the new atomspace
            (set! temp-rule
                (cog-new-link 'BindLink (cog-outgoing-set rule)))
            ;(display "temp-rule: ") (display temp-rule)(newline)
            (cog-new-link 'ListLink (cog-outgoing-set
                (mapConceptualizeString cleaned-text)))
            ;(display "********************  (prt) **********************\n")
            ;(prt)
            ;(display "********************  prt end ********************\n")

            ; Now unify the rule and listified string in the temp atomspace
            (set! bind-results (cog-bind temp-rule))
            ;(display "cog-bind temp-rule: ")(display bind-results)(newline)

            ; Now switch back to the orig atomspace and evaluate the
            ; behavior rule conssequence.
            (cog-set-atomspace! as-orig)
            ;(display (cog-atomspace))(newline)

            (set! consequent (gar bind-results))

            ; Need to get the equiv consequent atom of the temp atomspace bind
            ; result from the orig atomspace -- ugh, this is hacky.
            (if (cog-node? consequent)
                (set! consequent (cog-node
                    (cog-type consequent) (cog-name consequent))))

            (cog-evaluate! consequent)
            ;(cog-evaluate! (gar (cog-bind rule)))
        )
        #f
    )
	;(cog-pop-atomspace)
)

; Retrieve atomspace behavior rule with antecedent that contains atomese
; representation of the input string.
;(define (get-tree-with-antecedent input-str)
(define (get-tree-with-antecedent cleaned-text)
    ; TODO: For now just using a single result, but we should handle multiple
    ;       returned results.
    ; TODO: Allow for variable words
	; TODO: Match on multiple conditions (ie using OR)
    (let* (
    ;        (cleaned-text
    ;            ;(string-trim-both (cleanText (string-upcase input-str))))
    ;            (clean-text input-str))
           (results (findQueryPatterns cleaned-text)))
        ;(display "results:\n") (display results)
        (if (> (length (cog-outgoing-set results)) 0)
	        (gar results)
	        #f
	    )
    )
)

(define (clean-text input-str)
    (string-trim-both (cleanText (string-upcase input-str))))

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
    (display "\n(create-behavior-rule) \n    stimulus: ")(display stimulus)
        (display "    response: ")(display response)(newline)
    ;(set! atomese-string (string-to-atomese stimulus))

    ; TODO: we should check first make sure the response is a pre-defined behavior
	(set! new-rule
    	(BindLink
    		stimulus
			(DefinedPredicateNode  (string-downcase (cog-name (list-ref (cog-outgoing-set response) 0))))
		)
	)
	(display new-rule)
    (stv 1 1)
)


;-----------------------------------------------------------------
; Utils
; TODO: move to util file

; shortcuts
(define (incoming atom) (cog-incoming-set atom))
(define (prt) (cog-prt-atomspace))


;-----------------------------------------------------------------
; For testing
(define w (Concept "WHEN"))


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



