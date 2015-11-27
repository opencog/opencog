;
; Opencog cogserver module
;
(define-module (opencog cogserver))

; libguile-cogserver.so is located in /usr/local/lib/opencog
; libnlp-types.so is in /usr/local/lib/opencog/modules
(setenv "LTDL_LIBRARY_PATH" "/usr/local/lib/opencog:/usr/local/lib/opencog/modules")

(load-extension "libguile-cogserver" "opencog_cogserver_init")

; config path name is optional.
(define* (start-cogserver #:optional (config-path ""))
"
  start-cogserver [config-file]

  Start the cogserver, optionally specifying the config file to use.
  To stop the cogserver, just say stop-cogserver.
"
	;; Server falls over if the atom types are not loaded.
	(use-modules (opencog atom-types))
	(c-start-cogserver config-path)
)

; to stop the repl server..
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
