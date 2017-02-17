;
; sec.scm
;
; Stimulus Evaluation Check (SEC) entity from Process Component Model theory
;
; SEC are associated with a particular stimulus or the agent state. Thus, they
; do not have values on their own but only in association with a stimulus.

; Parameter specifying the initial default level for agent-state SECs
(define agent-state-sec-init-value .5)

; Agent State (iow system state)
;(define agent-state (Concept (string-append psi-prefix-str "agent-state")))

(define psi-sec-node (Concept (string-append psi-prefix-str "SEC")))

(define-public (psi-create-sec name initial-value)
"
	Create a new SEC

	name - name of the SEC
"
	(define sec
		(Concept (string-append psi-prefix-str name)))
	(Inheritance
		sec
		psi-sec-node)
	(psi-set-value! sec initial-value)
	; Set the baseline value as the initial-value
	(psi-set-baseline-value! sec initial-value)

	; For now, assuming the sec alone represents the overall system/agent state,
	; for consi1stency with psi var representations. Code below can be used if we
	; decide we want this represented explicitly as agent-state.
	; Create the agent/system-state variable for this sec
	;(psi-create-stimulus-sec agent-state sec agent-state-sec-init-value)
	; Dynamic variable name creation needs working out.
	;(define agent-state-var-name (string-append "agent-state-" name))
	;(eval `(define ,(string->symbol agent-state-var-name)
	;    ,(psi-create-stimulus-sec agent-state sec agent-state-sec-init-value)))
	;(export (string->symbol agent-state-var-name))

	sec)

(define-public (psi-create-stimulus-sec stimulus sec initial-value)
"
  Create a stimulus-SEC association and assign its initial value.
  A stimulus-SEC takes the form (List stimulus SEC) and its current value is
  stored in a StateLink:
	  (StateLinke (List stimulus SEC) (Number x))
  The value is assumed to be in [0,1].
"
	(define stimulus-sec
		(List
			stimulus
			sec))
	(psi-set-value! stimulus-sec initial-value)
	stimulus-sec)

(define-public (psi-get-secs)
	(cog-outgoing-set
		(cog-execute! (Get (Inheritance (Variable "$sec") psi-sec-node)))))

(define-public (psi-is-sec? atom)
	(member atom (psi-get-secs)))

; Todo: add variable names (?) and add getters for agent-state secs

; =============================================================================
; CREATE SECs

;(define secs (list
;		"novelty"
;		"goal-relevance"
;		"pleasantness"
;		"outcome-probability"
;		"surprise"
;		"agent-and-intention"
;		"control"
;		"power"
;		"adjustment"
;		"standards" ))

;(for-each (lambda (sec) (psi-create-sec sec)) secs)

(define novelty (psi-create-sec "novelty" agent-state-sec-init-value))
(define goal-relevance (psi-create-sec "goal-relevance" agent-state-sec-init-value))
(define pleasantness (psi-create-sec "pleasantness" agent-state-sec-init-value))
(define outcome-probability (psi-create-sec "outcome-probability" agent-state-sec-init-value))
(define surprise (psi-create-sec "surprise" agent-state-sec-init-value))
(define agent-and-intention (psi-create-sec "agent-and-intention" agent-state-sec-init-value))
(define control (psi-create-sec "control" agent-state-sec-init-value))
(define power (psi-create-sec "power" agent-state-sec-init-value))
(define adjustment (psi-create-sec "adjustment" agent-state-sec-init-value))
(define standards (psi-create-sec "standards" agent-state-sec-init-value))

; Temp for development, until this assignment happens in the create function
; Create Stimulus-SEC Associations
;(define agent-state-power
;       (psi-create-stimulus-sec agent-state power .5))
