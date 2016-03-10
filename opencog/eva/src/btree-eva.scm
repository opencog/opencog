;
; btree-eva.scm
;
; Eva behavior tree (for the Eva blender model animations).
;
; Runs a set of defined behaviors that express Eva's personality.
; This version integrates the OpenCog chatbot.
;
; The currently-defined behaviors include acknowledging new people who
; enter the room, rotating attention between multiple occupants of the
; room, falling asleep when bored (i.e. the room is empty), and acting
; surprised when someone leaves unexpectedly.
;
; HOWTO:
; Run the main loop:
;    (run)
; Pause the main loop:
;    (halt)
;
; Make sure that you did a `cmake` and `make install` first!
;
; Unit testing:  See `unit-test.scm` and also notes in `behavior.scm`.
;

(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog))

; Start the cogsserver.  It is used by the face-tracker to poke data
; into the atomspace.
(use-modules (opencog cogserver))
(start-cogserver "../scripts/opencog.conf")

; Load the behavior trees.
(use-modules (opencog exec))        ; needed for cog-evaluate! in put_atoms.py
(use-modules (opencog eva-model))   ; needed for defines in put_atoms.py
(use-modules (opencog eva-behavior))

; Load the Eva personality configuration.
; (display %load-path)
(add-to-load-path "../src")
(load-from-path "cfg-eva.scm") ;;; <<<=== See, its Eva here!

;; Call (run) to run the main loop, (halt) to pause the loop.
;; The main loop runs in its own thread.
(define (run) (behavior-tree-run))
(define (halt) (behavior-tree-halt))

; ---------------------------------------------------------
; Load the chat modules.
;
(use-modules (opencog nlp))
(use-modules (opencog nlp chatbot-eva))

; XXX remove the below when we get a chance.
; Must load the rulebase before running eva; see bug
; https://github.com/opencog/opencog/issues/2021 for details
; XXX fixme -- we should not need to load either relex2logic or
; the rules right here, since the code in this module does not depend
; directly on thes.
(use-modules (opencog nlp relex2logic))
(load-r2l-rulebase)

; ---------------------------------------------------------
; Run the hacky garbage collection loop.
(run-behavior-tree-gc)

; Silence the output.
*unspecified*

;; Actually set it running, by default.
;; Actually, don't, just right now.
; (all-threads)  ; print out the curent threads.
; (run)
