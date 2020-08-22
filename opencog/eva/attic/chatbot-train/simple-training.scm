; simple-training.scm
;
; Experimental, simple training for hanson bots--on-the-fly creation of new
; behavior rules through natural language interaction with the robot.
;
; Starting off with simple, rigid command template to learn new stimulus and
; associated response behavior.
; Example:
; Human "When I say 'Are you bored' then yawn."
; Bot: "Okay, when you say are you bored, then I will yawn."
; Human "Are you bored?"
; Bot: <yawns>
;
; Requires installation of opencog/ros-behavior-scripting

; TODO: probably can take out eva-behavior module when express.scm is moved out
;       of that module.
(use-modules (opencog) (opencog query) (opencog exec) (opencog eva-model)
    (opencog eva-behavior))

; load kino's atomese string match code
(load "../aiml2oc/aiml2oc_guile/code/OpenCogAimlReply1.scm")

; load config file for chatbot (eva)
(load-from-path "opencog/eva-behavior/cfg-eva.scm")

(load "behavior-defs.scm")
(load "behavior-rules.scm")

; TODO: multiple globnode values are not matching (Github issue created #724)

;-----------------------------------------------------------------
; The main training rule template for natural language training

; Todo: Or / Choice links not working in antecedant for recongition, so doing a
; couple different rules for now as a workaround. Potential solution: also
; search for OR at top level with cog-recognize

; "When I say * you *"
(define training-rule
    (BindLink
        (ListLink
            (ConceptNode "WHEN")
            (ConceptNode "I")
            (ConceptNode "SAY")
            (GlobNode "$stimulus")
            (ConceptNode "THEN")
            (GlobNode "$response"))
        (Evaluation
            (GroundedPredicateNode "scm:create-behavior-rule")
            (ListLink
                (List (GlobNode "$stimulus"))
                (List (GlobNode "$response"))))))

; Removing this for now - need to figure out how to stop the pattern below from
; matching with the pattern above. Also need to figure out how to deal with the
; situation of a words in the stimuls also being in the training pattern. E.g.,
; if the training pattern is "When I say * you *" and then the word "you" gets
; used in the stimilus.
; "When I say * then you *"
#!
(define training-rule
    (BindLink
        (ListLink
            (ConceptNode "WHEN")
            (ConceptNode "I")
            (ConceptNode "SAY")
            (GlobNode "$stimulus")
            (ConceptNode "THEN")
            (ConceptNode "YOU")
            (GlobNode "$response"))
        (Evaluation
            (GroundedPredicateNode "scm:create-behavior-rule")
            (ListLink
                (List (GlobNode "$stimulus"))
                (List (GlobNode "$response"))))))
!#

;-----------------------------------------------------------------
; Execution of behaviors based on string stimulus using AIML-style string
; matching.

; This is the main function that controls execution of a behavior rule based on
; a stimulus input string, if there is a match with a rule conditional.
(define (execute-behavior-with-stimulus input-str)
    (define rule)
    (define bind-results)
    (define eval-results)
    (define as-orig)
    (define temp-rule)
    (define consequent)
    (define result)
	(define listified-string (mapConceptualizeString (clean-text input-str)))

    (set! rule (get-tree-with-antecedent listified-string))
    (if rule
        (begin
            (display "rule: ")(display rule)(newline)

            ; Need to unify the variables in the rule with the input without
            ; matching to existing phrases that may be in the atomspace.
            ; Create and set a temp atomspace
            (set! as-orig (cog-set-atomspace! (cog-new-atomspace)))
            ; Add the rule and the listified input to the new atomspace
            (set! temp-rule
                (cog-new-link 'BindLink (cog-outgoing-set rule)))
            ;(display "temp-rule: ") (display temp-rule)(newline)
            (cog-new-link 'ListLink (cog-outgoing-set listified-string))

            ; Now unify the rule and listified string in the temp atomspace
            (set! bind-results (cog-execute! temp-rule))
            ;(display "cog-execute! temp-rule: ")(display bind-results)(newline)

            ; Now switch back to the orig atomspace and evaluate the
            ; behavior rule conssequence.
            (cog-set-atomspace! as-orig)
            (set! consequent (gar bind-results))

            ; Need to get the equiv consequent atom of the temp atomspace bind
            ; result from the orig atomspace or else it won't evaluate -- ugh,
            ; this is hacky.
            (if (cog-node? consequent)
                (set! consequent (cog-node
                    (cog-type consequent) (cog-name consequent))))

            (set! result (cog-evaluate! consequent))
        )
        (set! result #f)
    )
    ; Remove the temp needed listified string
    (cog-extract! listified-string)
	result
)

; Retrieve atomspace behavior rule with antecedent that contains atomese
; representation of the input string.
(define (get-tree-with-antecedent listified-string)
    ; TODO: For now just using a single result, but we should handle multiple
    ;       returned results.
	; TODO: Match on multiple conditions (ie using OR)
	;create temp child atomspace for temporarily needed atoms
	(cog-push-atomspace)
    (let* ((query-pattern (PatternLink
                               (BindLink
                                  listified-string
                                  (VariableNode "$impl"))))
           (results (cog-recognize query-pattern))
          )
        (cog-pop-atomspace)
        ;(display "results:\n") (display results)
        (if (> (length (cog-outgoing-set results)) 0)
	        (gar results)
	        #f
	    )
    )
)

(define (clean-text input-str)
    (string-trim-both (cleanText (string-upcase input-str))))

; shortcut method to pass input string for processing
(define (say input-str)
    (execute-behavior-with-stimulus input-str))

; Creates "listified" atomese reresentation of input strings in the format:
;   ListLink
;     ConceptNode "ANDROID"
;     ConceptNode "LIVES"
;     ConceptNode "MATTER"
(define (string-to-atomese input)
    (define atomese-string)
    (define stim-and-response)
    (define cleaned-text
        (string-trim-both (cleanText (string-upcase input))))
    (set! atomese-string (genQueryPattern cleaned-text))
    (display "atomese-string: ")(atomese-string)(newline)

)


(define (delistify ll)
	(define concept-words (cog-outgoing-set ll))
	(define first (cog-name (list-ref concept-words 0)))
	(set! concept-words (list-tail concept-words 1))
	(string-downcase
		(fold (lambda (new prev) (string-append prev " " (cog-name new)))
			first
			concept-words)))



;-----------------------------------------------------------------
; Creates new behavior rule in atomese based on a given simulus and response
(define (create-behavior-rule stimulus response)
; TODO: Create feedback response that something was learned
    ; create new behavior rule with text input stimulus and behavior response
    (define new-rule)
    (define atomese-string)
    (display "\n(create-behavior-rule) \n    stimulus: ")(display stimulus)
        (display "    response: ")(display response)(newline)

    ; TODO: we should check first make sure the response is a pre-defined behavior
	(set! new-rule
    	(BindLink
    		stimulus
			(DefinedPredicateNode  (delistify response))
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
(define (root atom) (cog-get-root atom))


;-----------------------------------------------------------------
; For testing
(define w (Concept "WHEN"))



; TODO: ask linas about these
; Approach to pull the training rule from the atomspace dynamically rather than
; relying on the scheme var.
; maybe the DefinedType atom is being replaced in the cog-satisfy
; TODO: let's try cog-get-partner instead
; Check if an atom is the training rule
(define (training-rule? atom)
    (cog-satisfy
        (SatisfactionLink
            (DefineLink
                (DefinedType "training-rule")
                atom))))

(define (training-rule2 atom)
    (cog-get
        (GetLink
            (DefineLink
                (DefinedType "training-rule")
                atom))))



