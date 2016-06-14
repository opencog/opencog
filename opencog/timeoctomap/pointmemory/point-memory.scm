(setenv "LTDL_LIBRARY_PATH"
    (if (getenv "LTDL_LIBRARY_PATH")
        (string-append (getenv "LTDL_LIBRARY_PATH")
            ":/usr/local/lib/opencog:/usr/local/lib/opencog/modules")
        "/usr/local/lib/opencog:/usr/local/lib/opencog/modules"))

(define-module (opencog ato pointmem))

(use-modules (srfi srfi-1) (opencog) )

(load-extension "libpoint_memory" "opencog_ato_pointmem_init")


