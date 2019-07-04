
(use-modules (opencog) (opencog nlp) (opencog atom-types))

(define-module (opencog patternminer))

(use-modules (opencog oc-config))
; This loads the pattern-miner atom types.
(load-extension (string-append opencog-ext-path-pattern-miner "libguile-patternminer") "opencog_patternminer_init")
