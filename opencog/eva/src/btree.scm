;
; btree.scm
;
; Sophia behavior tree, implemented in the atomspace.
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
; Unit testing:  See notes in `behavior.scm`.
;

(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog))

; Start the cogsserver and it's RestAPI.  I don't know why this is
; needed; Vytas added this code earlier in a different file.
(use-modules (opencog cogserver))
(start-cogserver "../scripts/opencog.conf")
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


; (display %load-path)
(add-to-load-path "../src")
(load-from-path "cfg-tools.scm")
(load-from-path "sophia-cfg.scm") ;;; <<<=== See, its Sophia here!
(use-modules (opencog eva-behavior))

; (use-modules (opencog logger))
; (cog-logger-set-stdout #t)

;; Run the main loop (in a new thread)
;; Call (run) to run the loop, (halt) to pause the loop.
(define (run) (behavior-tree-run))
(define (halt) (behavior-tree-halt))

; Run the hacky garbage collection loop.
(run-behavior-tree-gc)

; Silence the output.
(TrueLink)

;; Actually set it running
(all-threads)
(run)
