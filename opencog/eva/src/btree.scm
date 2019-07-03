;
; btree.scm
;
; XXX FIXME ... I think this blob of code is obsolete ... I think
; that the current interfaces are in `btree-psi.scm`.  So remove this
; file once we are clear on this.
;
; Sophia behavior tree (for the Sophia blender model animations).
;
; Runs a set of defined behaviors that express Sophia's personality.
;
; The currently-defined behaviors include acknowledging new people who
; enter the room, rotating attention between multiple occupants of the
; room, falling asleep when bored (i.e. the room is empty), and acting
; surprised when someone leaves unexpectedly.
;
; The behaviors defined here are integrated with the chatbot in an
; ad-hoc way, with no real coordination between chat and behavior.
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
(load "time-map.scm") ;;; octomap for faces
; Start the cogsserver.  It is used by the face-tracker to poke data
; into the atomspace.
(use-modules (opencog cogserver))
(start-cogserver "../scripts/opencog.conf")

;;; Start the cogserver's RestAPI.  Vytas was using this to experiment
;;; with the opencog graph visualizer. But its broken on Vytas demo
;;; machine, just right now.
;;(use-modules (opencog python))
;;(python-eval "
;;try:
;;    from opencog.cogserver import get_server_atomspace
;;    from web.api.restapi import Start as RestAPI
;;    atspace = get_server_atomspace()
;;    set_common_shared_atomspace(atspace)
;;    RestAPI().run(None, atomspace=atspace)
;;except:
;;    print('Rest API can't be started. Check Python configuration')
;;")

; Load the behavior trees.
(use-modules (opencog eva-behavior))

; Load the Sophia personality configuration.
; (display %load-path)
(add-to-load-path "../src")
(load-from-path "cfg-sophia.scm") ;;; <<<=== See, its Sophia here!

;; Call (run) to run the main loop, (halt) to pause the loop.
;; The main loop runs in its own thread.
(load-from-path "old-tree.scm")
(define (run) (behavior-tree-run))
(define (halt) (behavior-tree-halt))

; ---------------------------------------------------------
; Load the chat modules.
;
;(use-modules (opencog nlp))
;(use-modules (opencog nlp chatbot-eva))

; Work-around to weird bug: must load relex2logic at the top level.
;(use-modules (opencog nlp relex2logic))

; Work-around to circular dependency: define `dispatch-text` at the
; top level of the guile execution environment.
(define-public (dispatch-text TXT-ATOM)
"
  dispatch-text TXT-ATOM

  Pass the TXT-ATOM that STT heard into the OpenCog chatbot.
"
;	(call-with-new-thread
		; Must run in a new thread, else it deadlocks in python,
		; since the text processing results in python calls.
		; (lambda () (process-query "luser" (cog-name TXT-ATOM)))
		; (lambda () (grounded-talk "luser" (cog-name TXT-ATOM)))
		; XXX Need to ... auuh load the chatbot...
;		(lambda () (chat TXT-ATOM))
;	)
	(stv 1 1)
)

; ---------------------------------------------------------

; Run the hacky garbage collection loop.
(run-behavior-tree-gc)

; Silence the output.
(TrueLink)

;; Actually set it running, by default.
(all-threads)  ; print out the curent threads.
(run)
