;
; Opencog cogserver module
;

(define-module (opencog cogserver))

(use-modules (opencog oc-config))

; Path to libguile-cogserver.so is set up in the opencog module.
(load-extension (string-append opencog-ext-path-cogserver "libguile-cogserver") "opencog_cogserver_init")

; config path name is optional.
(define* (start-cogserver #:optional (config-path ""))
"
  start-cogserver [config-file]

  Start the cogserver, optionally specifying the config file to use.
  To stop the cogserver, just say stop-cogserver.
"
	(c-start-cogserver (cog-atomspace) config-path)
)

; To stop the repl server..
(use-modules (system repl server))

; Similar to above
(define (stop-cogserver)
"
  stop-cogserver

  Stop the cogserver.
"
	; The start-cogserver also starts a repl shell on port 18001
	; so we stop that, here ...
	(stop-server-and-clients!)
	(c-stop-cogserver)
)

(export start-cogserver)
(export stop-cogserver)
