;; URE Configuration file for the pattern miner, must be loaded first
;; before any pattern mining can take place.

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Load required modules and utils ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(use-modules (opencog))
(use-modules (opencog rule-engine))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define PLN rule-based system ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define pm-rbs (ConceptNode "pattern-miner"))
(InheritanceLink
   pm-rbs
   (ConceptNode "pattern-miner")
)

;; Define pln-fc and pln-bc for convenience 
(define (pm-fc source) (cog-fc pm-rbs source))
(define (pm-bc target) (cog-bc pm-rbs target))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules (use load for relative path w.r.t. to that file)
(add-to-load-path ".")
(define rule-files (list "rules/shallow-abstraction.scm"
                         "rules/specialization.scm"))
(for-each load-from-path rule-files)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; List the rules and their weights.
(define rules (list shallow-abstraction-rule-name
                    specialization-rule-name))

; Associate rules to PLN
(ure-add-rules pm-rbs rules)

;;;;;;;;;;;;;;;;;;;;;
;; Other paramters ;;
;;;;;;;;;;;;;;;;;;;;;

; Termination criteria parameters
(ure-set-num-parameter pm-rbs "URE:maximum-iterations" 10)

;; Complexity penalty
(ure-set-num-parameter pm-rbs "URE:BC:complexity-penalty" 1)
