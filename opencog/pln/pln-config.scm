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
(define (pln-fc source) (cog-fc pln-rbs source))
(define (pln-bc target) (cog-bc pln-rbs target))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

; Load the rules (use load for relative path w.r.t. to that file)
(define config-dir (dirname (current-filename)))
(define (prepend-config-dir fp) (string-append config-dir "/" fp))
(define rule-files (list "rules/term/deduction.scm"
                         "rules/wip/modus-ponens.scm"))
(for-each (lambda (fp) (load (prepend-config-dir fp))) rule-files)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; List the rules and their weights.
(define rules (list (list deduction-inheritance-rule-name 1)
                    (list modus-ponens-implication-rule-name 1))
)

; Associate rules to PLN
(ure-add-rules pln-rbs rules)

;;;;;;;;;;;;;;;;;;;;;
;; Other paramters ;;
;;;;;;;;;;;;;;;;;;;;;

; Termination criteria parameters
(ure-set-num-parameter pln-rbs "URE:maximum-iterations" 20)

; Attention allocation (0 to disable it, 1 to enable it)
(ure-set-fuzzy-bool-parameter pln-rbs "URE:attention-allocation" 0)
