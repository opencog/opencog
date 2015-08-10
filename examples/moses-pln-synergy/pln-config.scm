;
; URE Configuration file for PLN
;
; Before running any PLN inference you must load that file in the
; AtomSpace
;
; In order to add new rules you need to hack this file in 2 places
;
; 1. In the Load rules section, to add the file name where the rule is
; defined (see define rule-files).
;
; 2. In the Associate rules to PLN section, to add the name of the
; rule and its weight (see define rules).

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Load required modules and utils ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(use-modules (opencog))
(use-modules (opencog rule-engine))

(load-from-path "utilities.scm")
(load-from-path "av-tv.scm")
(load-from-path "rule-engine-utils.scm")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define PLN rule-based system ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define pln-rbs (ConceptNode "PLN"))
(InheritanceLink
   pln-rbs
   (ConceptNode "URE")
)

; Define pln-fc and pln-bc for convenience 
(define (pln-fc source) (cog-fc source pln-rbs))
(define (pln-bc target) (cog-bc target pln-rbs))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

; Load the rules (use load for relative path w.r.t. to that file)
(define rule-files (list "pln-rules/deduction.scm"
                         "pln-rules/modus-ponens.scm"
                         "pln-rules/implication-or.scm"
                         "pln-rules/equivalence-transformation-rule.scm"
                         "pln-rules/hack.scm"
                   )
)
(for-each load rule-files)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; List the rules and their weights.
(define rules (list (list pln-rule-deduction 1)
                    (list pln-rule-modus-ponens 1)
                    (list pln-rule-equivalence-transformation 1)
              )
)

; Associate rules to PLN
(ure-add-rules pln-rbs rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

; Termination criteria parameters
(ure-set-num-parameter pln-rbs "URE:maximum-iterations" 20)

; Attention allocation (0 to disable it, 1 to enable it)
(ure-set-fuzzy-bool-parameter pln-rbs "URE:attention-allocation" 0)
