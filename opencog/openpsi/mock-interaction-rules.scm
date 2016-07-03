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

(define speech->power
	(psi-create-interaction-rule speech increased agent-state-power .5))

(define power->voice
	(psi-create-interaction-rule agent-state-power changed voice-width 1))

(define power->arousal
	(psi-create-interaction-rule agent-state-power changed arousal .1))

(define arousal->voice
	(psi-create-interaction-rule arousal changed voice-width -.9))

