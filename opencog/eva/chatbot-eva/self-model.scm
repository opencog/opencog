;
; self-model.scm
;
; Model of the robot-self, in the atomspace.
;
; In order for the robot to be able to talk about itself, it must have
; some amount of self-awareness; specifically, of its current physical
; state (head and eye position) and some memory of its most recent
; actions. That model-of-self must lie in the atomspace, where it can
; be examined during the course of linguistic interaction.  It is a
; "model" -- it is NOT the actual blender rig; it is not the actual
; mechanical motor positions and velocities.
;
; This is meant to be exemplary: there also needs to be a model of
; Eva's environment (including the people that she sees in that
; environment), so that she can also talk about that.
;--------------------------------------------------------------------

; The primary physical self-model is in the eva/model directory.
; Use that.

(use-modules (opencog eva-model))

;--------------------------------------------------------------------
