
(use-modules (opencog) (opencog nlp) (opencog atom-types))

(define-module (opencog patternminer))

; This loads the pattern-miner atom types.
(load-extension "libguile-patternminer" "opencog_patternminer_init")
