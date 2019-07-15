;;
;; OpenCog Miner module
;;
(define-module (opencog miner))

(use-modules (opencog oc-config))
;; This loads the miner atom types.
(load-extension (string-append opencog-ext-path-miner "libguile-miner") "opencog_miner_init")

;; Load miner utils

;; TODO: this doesn't correspond to the local path, it needs to be
;; installed, maybe we should do like atomspace/scm/opencog so that
;; the installed path matches the local path
(load-from-path "opencog/miner/miner-utils.scm")
(export-miner-utils)
