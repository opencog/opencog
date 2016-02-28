;
; btree.scm
;
; Sophia behavior tree.
;
; Defines a set of behaviors that express Sophia's personality. The
; currently-defined behaviors include acknowledging new people who enter
; the room, rotating attention between multiple occupants of the room,
; falling asleep when bored (i.e. the room is empty), and acting
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

; Start the cogserver's RestAPI.  I don't know why this is
; needed; Vytas added this code earlier in a different file.
(use-modules (opencog python))
(python-eval "
try:
    from opencog.cogserver import get_server_atomspace
    from web.api.restapi import Start as RestAPI
    atspace = get_server_atomspace()
    set_common_shared_atomspace(atspace)
    RestAPI().run(None, atomspace=atspace)
except:
    print 'Rest API can't be started. Check Python configuration'
")

; Load the behavior trees.
(use-modules (opencog eva-behavior))

; Load the Sophia personality configuration.
; (display %load-path)
(add-to-load-path "../src")
(load-from-path "cfg-tools.scm")
(load-from-path "sophia-cfg.scm") ;;; <<<=== See, its Sophia here!

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
