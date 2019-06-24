;
; btree-psi.scm
;
; Sophia OpenPsi behavior action selection (for the Sophia blender
; model animations).
;
; Runs a set of defined behaviors that express Sophia's personality.
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
(use-modules (opencog))
(use-modules (opencog logger))
(use-modules (opencog eva-model))
; Start the cogsserver.  It is used by the face-tracker to poke data
; into the atomspace.
(use-modules (opencog cogserver))
(start-cogserver)

; Load the behavior trees associated psi-rules.
(use-modules (opencog openpsi))
(use-modules (opencog eva-behavior))

; Load the sophia's personality configuration.
(load-sophia-config)

; Module from ros-behavior-scripting
(use-modules (opencog movement))
(start-ros-movement-node)

;; Call (run) to run the main loop, (halt) to pause the loop.
;; The main loop runs in its own thread.
(define (run) (psi-run-per-demand))
(define (halt) (psi-halt))

; ---------------------------------------------------------
; Load the chat modules.
;
(use-modules (opencog nlp))
(use-modules (opencog nlp chatbot))

; Work-around to weird bug: must load relex2logic at the top level.
(use-modules (opencog nlp relex2logic))

; Work-around to circular dependency: define `dispatch-text` at the
; top level of the guile execution environment.
(define-public (dispatch-text TXT-ATOM)
"
  dispatch-text TXT-ATOM

  Pass the TXT-ATOM that STT heard into the OpenCog chatbot.
"
   (call-with-new-thread
		; Must run in a new thread, else it deadlocks in python,
		; since the text processing results in python calls.
      ; (lambda () (process-query "luser" (cog-name TXT-ATOM)))
      ; (lambda () (grounded-talk "luser" (cog-name TXT-ATOM)))
      (lambda () (chat (cog-name TXT-ATOM)))
   )
   (stv 1 1)
)

; Configure the logger
(define (configure-loggers LOG-LEVEL)
"
  configure-loggers LOG-LEVEL

  Set the loggers to the same level and separate their logs between runs.
  For each call of this function, the logs are created in
  /tmp/<current-filename>/<module-name>-Year-Month-Day-Hour-Minute-Second.log
"
  (define log-dir (format #f "/tmp/~a" (basename (current-filename))))
  (define z-time (strftime "%F-%H-%M-%S" (localtime (current-time))))
  (define (configure-logger logger name)
    (cog-logger-set-level! logger LOG-LEVEL)
    (cog-logger-set-stdout! logger #f)
    (cog-logger-set-filename! logger
      (format #f "~a/~a-~a.log" log-dir name z-time)))

  (if (not (file-exists? log-dir)) (mkdir log-dir))

  ; Configure openpsi logger
  (configure-logger (psi-get-logger) "openpsi")

  ; Configure eva-model logger
  (configure-logger (eva-get-logger) "eva")

  ; Configure opencog default logger
  (let ((oc-log-file (format #f "~a/opencog-~a.log" log-dir z-time)))
    (cog-logger-set-level! LOG-LEVEL)
    (cog-logger-set-filename! oc-log-file)
    (cog-logger-set-stdout! #f)
  )
)

(configure-loggers "debug")

; Silence the output.
*unspecified*
