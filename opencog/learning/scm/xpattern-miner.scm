;;
;; OpenCog XPattern Miner module
;;
(define-module (opencog xpattern-miner))

;; We need this to set the LTDL_LIBRARY_PATH
(use-modules (opencog))

;; This loads the xpattern-miner atom types.
(load-extension "libguile-xpatternminer" "opencog_xpatternminer_init")
