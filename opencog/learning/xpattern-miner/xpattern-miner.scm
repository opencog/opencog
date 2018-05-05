;;
;; OpenCog XPattern Miner module
;;
(define-module (opencog xpattern-miner))

;; We need this to set the LTDL_LIBRARY_PATH
(use-modules (opencog))

;; This loads the xpattern-miner atom types.
(load-extension "libguile-xpatternminer" "opencog_xpatternminer_init")

;; Load xpattern miner utils

;; TODO: this doesn't correspond to the local path, it needs to be
;; installed, maybe we should do like atomspace/scm/opencog so that
;; the installed path matches the local path
(load-from-path "opencog/xpattern-miner/xpattern-miner-utils.scm")
(export-xpattern-miner-utils)
