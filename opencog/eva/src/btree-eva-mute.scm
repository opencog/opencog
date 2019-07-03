;
; btree-eva-mute.scm
;
; Eva behavior tree (for the Eva blender model animations).
;
; Runs a set of defined behaviors that express Eva's personality.
; This is the "mute" (silent) version; it does not involve OpenCog
; chatbot (it does use the ad-hoc chotbot coordination with external
; chatbots)
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
(use-modules (opencog))

; Start the cogsserver.  It is used by the face-tracker to poke data
; into the atomspace.
(use-modules (opencog cogserver))
(start-cogserver "../scripts/opencog.conf")

; Load the behavior trees.
(use-modules (opencog eva-behavior))

(use-modules (opencog movement))
(start-ros-movement-node)

; Load the Eva personality configuration.
; (display %load-path)
(add-to-load-path "../src")
(load-from-path "cfg-eva.scm") ;;; <<<=== See, its Eva here!
(load-from-path "old-tree.scm")

;; Call (run) to run the main loop, (halt) to pause the loop.
;; The main loop runs in its own thread.
(define (run) (behavior-tree-run))
(define (halt) (behavior-tree-halt))

; Run the hacky garbage collection loop.
(run-behavior-tree-gc)

; Silence the output.
(TrueLink)

;; Actually set it running, by default.
(all-threads)  ; print out the curent threads.
(run)
