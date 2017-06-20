;
; Opencog cogserver module
;

; Old CentOS-based systems use lib64
(define path "/usr/local/lib/opencog:/usr/local/lib64/opencog")

(setenv "LTDL_LIBRARY_PATH"
    (if (getenv "LTDL_LIBRARY_PATH")
        (string-append (getenv "LTDL_LIBRARY_PATH") ":" path)
        path))

(define-module (opencog cogserver))

; libguile-cogserver.so is located in /usr/local/lib/opencog
(load-extension "libguile-cogserver" "opencog_cogserver_init")

;; Server falls over if the atom types are not loaded.
(use-modules (opencog) (opencog atom-types))

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
