;;
;; OpenCog Miner module
;;
(define-module (opencog miner))

;; We need this to set the LTDL_LIBRARY_PATH
(use-modules (opencog))

;; This loads the miner atom types.
(load-extension "libguile-miner" "opencog_miner_init")

;; Load miner utils

;; TODO: this doesn't correspond to the local path, it needs to be
;; installed, maybe we should do like atomspace/scm/opencog so that
;; the installed path matches the local path
(load-from-path "opencog/miner/miner-utils.scm")
(export-miner-utils)
