;
; btree.scm
;
; Behavior tree, implemented in the atomspace.
;
; Defines a set of behaviors that express Eva's personality. The
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
; Unit testing:
; The various predicates below can be manually unit tested by manually
; adding and removing new visible faces, and then manually invoking the
; various rules. See faces.scm for utilities.
;
; Basic examples:
;    Manually insert a face: (make-new-face id)
;    Remove a face: (remove-face id)
;    Etc.: (show-room-state) (show-interaction-state) (show-visible-faces)
;
; Manually unit test the new-arrival sequence.  You can do this without
; an attached camera; she won't track the face location, but should respond.
;    (make-new-face "42")
;    (cog-evaluate! (DefinedPredicateNode "New arrival sequence"))
;    (show-acked-faces)
;    (show-room-state)
;    (show-interaction-state)
;    (cog-evaluate! (DefinedPredicateNode "Interact with people"))
;
; Unit test the main interaction loop:
;    (run)  ; start the behavior tree running. Should print loop iteration.
;    (make-new-face "42")  ; Should start showing expressions, gestures.
;    (remove-face "42")  ; Should retur to neutral.
;    (halt) ; Pause the loop iteration; (run) will start it again.
;
; Unit test chatbot:
;    (State chat-state chat-start) ; to simulate having it talk.
;    (State chat-state chat-stop)  ; to signal that talking has stopped.

(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog) (opencog query) (opencog exec))
(use-modules (opencog atom-types))
(use-modules (opencog cogserver))
(start-cogserver "../scripts/opencog.conf")

(system "echo \"py\\n\" | cat - atomic.py |netcat localhost 17020")
; (system "echo \"py\\n\" | cat - atomic-dbg.py |netcat localhost 17020")

(load-from-path "utilities.scm")

; (display %load-path)
(add-to-load-path "../src")
(load-from-path "faces.scm")
(load-from-path "cfg-tools.scm")
(load-from-path "sophia-cfg.scm")
(load-from-path "self-model.scm")
(load-from-path "orchestrate.scm")
(load-from-path "express.scm")
(load-from-path "behavior.scm")

; (use-modules (opencog logger))
; (cog-logger-set-stdout #t)

;; Run the loop (in a new thread)
;; Call (run) to run the loop, (halt) to pause the loop.
;; line 297 -- self.tree.next()
(define (run) (behavior-tree-run))
(define (halt) (behavior-tree-halt))

; Sigh. Perform manual garbage collection. This really should be
; automated. XXX TODO. (Can we get ECAN to do this for us?)
(define (run-atomspace-gc)
	(define (free-stuff)
		(sleep 1)
		(cog-map-type (lambda (a) (cog-delete a) #f) 'SetLink)
		(cog-map-type (lambda (a) (cog-delete a) #f) 'ListLink)
		(cog-map-type (lambda (a) (cog-delete a) #f) 'NumberNode)
		(cog-map-type (lambda (a) (cog-delete a) #f) 'ConceptNode)
		(free-stuff)
	)
	(call-with-new-thread free-stuff)
)

; Run the gc too...
(run-atomspace-gc)

;
; Silence the output.
(TrueLink)

;; Actually set it running
(all-threads)
(run)
