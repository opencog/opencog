;
; XPattern Miner Engine module
;
(define-module (opencog xpattern-miner))

; We need this to set the LTDL_LIBRARY_PATH
(use-modules (opencog))

(load-extension "libruleengine" "opencog_ruleengine_init")

(load-from-path "opencog/xpattern-miner/xpattern-miner-utils.scm")
(export-xpattern-miner-utils)
