;
; Opencog cogserver module
;
(define-module (opencog cogserver))


; Hack LD path, but argh, this doesn't work!
; (setenv "LD_LIBRARY_PATH" "/usr/local/lib/opencog/modules")

(load-extension "libguile-cogserver" "opencog_cogserver_init")

; config path name is optional.
(define* (start-cogserver #:key config-path)
	(if (not config-path)
		(c-start-cogserver "")
		(c-start-cogserver config-path)
	)
)

(export start-cogserver)
