(setenv "LTDL_LIBRARY_PATH"
    (if (getenv "LTDL_LIBRARY_PATH")
        (string-append (getenv "LTDL_LIBRARY_PATH")
            ":/usr/local/lib/opencog:/usr/local/lib/opencog/modules")
        "/usr/local/lib/opencog:/usr/local/lib/opencog/modules"))

(define-module (opencog attention))

(load-extension "libguile-attention" "opencog_ecan_init")
