;
; mock-interaction-rules.scm
;
; mock interaction rules for openpsi dynamics development

; The following change-predicate types have been defined in
; interaction-rules.scm:
;(define changed "changed")
;(define increased "increased")
;(define decreased "decreased")

; ===========================================================================
; Below are bogus mock OpenPsi interaction rules for dev purposes


(define power->voice
	(psi-create-interaction-rule agent-state-power changed voice-width 1))

(define power->arousal
	(psi-create-interaction-rule agent-state-power changed arousal .1))

(define arousal->voice
	(psi-create-interaction-rule arousal changed voice-width -.9))


; new face -> increased arousal
(define new-face->arousal
	(psi-create-interaction-rule new-face increased arousal .5))

; speech giving starts -> increased power
(define speech->power
	(psi-create-interaction-rule speech-giving-starts increased
		agent-state-power .5))

; positive sentiment dialog -> increased valence
(define pos-dialog->valence
	(psi-create-interaction-rule positive-sentiment-dialog increased valence
		.5))

; negative sentiment dialog -> decreased valence
(define pos-dialog->valence
	(psi-create-interaction-rule negative-sentiment-dialog increased valence
		-.5))



