;
; mock-interaction-rules.scm
;
; mock interaction rules for openpsi dynamics development

; The following change-predicate types have been defined in
; interaction-rule.scm:
;(define changed "changed")
;(define increased "increased")
;(define decreased "decreased")

; ===========================================================================
; Below are bogus mock OpenPsi interaction rules for dev purposes


(define power->voice
	(psi-create-interaction-rule power changed voice-width 1))

(define power->arousal
	(psi-create-interaction-rule power changed arousal .1))

(define arousal->voice
	(psi-create-interaction-rule arousal changed voice-width -.9))


; new face -> increased arousal
(define new-face->arousal
	(psi-create-interaction-rule new-face increased arousal .5))

; speech giving starts -> increased power
(define speech->power
	(psi-create-interaction-rule speech-giving-starts increased
		power .5))

; positive sentiment dialog -> increased pos-valence
(define pos-dialog->pos-valence
	(psi-create-interaction-rule positive-sentiment-dialog changed pos-valence
		.5))

; positive sentiment dialog -> decreased neg-valence
(define pos-dialog->neg-valence
	(psi-create-interaction-rule positive-sentiment-dialog changed neg-valence
		-.5))


; negative sentiment dialog -> increased neg-valence
(define pos-dialog->neg-valence
	(psi-create-interaction-rule negative-sentiment-dialog changed neg-valence
		.5))

; negative sentiment dialog -> decreased pos-valence
(define pos-dialog->pos-valence
	(psi-create-interaction-rule negative-sentiment-dialog changed pos-valence
		-.5))



