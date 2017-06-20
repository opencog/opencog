; Old CentOS-based systems use lib64
(define path "/usr/local/lib/opencog:/usr/local/lib64/opencog")

(setenv "LTDL_LIBRARY_PATH"
    (if (getenv "LTDL_LIBRARY_PATH")
        (string-append (getenv "LTDL_LIBRARY_PATH") ":" path)
        path))

(define-module (opencog patternminer))
(use-modules (opencog) (opencog nlp) (opencog atom-types))

; This loads the pattern-miner atom types.
(load-extension "libguile-patternminer" "opencog_patternminer_init")
