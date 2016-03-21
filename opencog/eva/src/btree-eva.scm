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
(use-modules (opencog query))  ; XXX work-around relex2logic bug

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

; Work-around to weird bug: must load relex2logic at the top level.
(use-modules (opencog nlp relex2logic))

; Work-around to circular dependency: define `dispatch-text` at the
; top level of the guile executation environment.
(define-public (dispatch-text txt)
"
  dispatch-text TEXT

  Pass the TEXT that STT heard into the OpenCog chatbot.
"
   (call-with-new-thread
		; Must run in a new thread, else it deadlocks in python,
		; since the text processing results in python calls.
      ; (lambda () (process-query "luser" (cog-name txt)))
      (lambda () (grounded-talk "luser" (cog-name txt)))
   )
   (stv 1 1)
)

; ---------------------------------------------------------
; Run the hacky garbage collection loop.
(run-behavior-tree-gc)

; Silence the output.
*unspecified*

;; Actually set it running, by default.
;; Actually, don't, just right now.
; (all-threads)  ; print out the curent threads.
; (run)
