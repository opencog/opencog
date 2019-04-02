;;
;; URE Configuration file for PLN
;;
;; Before running any PLN inference you must load that file in the
;; AtomSpace
;;
;; In order to add new rules you need to hack this file in 2 places
;;
;; 1. In the Load rules section, to add the file name where the rule is
;; defined (see define rule-files).
;;
;; 2. In the Associate rules to PLN section, to add the name of the
;; rule and its weight (see define rules).

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Load required modules and utils ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(use-modules (opencog))
(use-modules (opencog ure))

;XXX This is bad and broken and wrong; one should not try to bypass the
; scheme module system like this, its just asking for carpet burns.
(load-from-path "utilities.scm")
(load-from-path "av-tv.scm")
(load-from-path "opencog/ure/ure-utils.scm")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define PLN rule-based system ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define pln-rbs (ConceptNode "PLN"))

;; Define pln-fc and pln-bc for convenience
(define (pln-fc source) (cog-fc pln-rbs source))
(define (pln-bc target) (cog-bc pln-rbs target))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules. Either w.r.t this file path
(add-to-load-path "../../../opencog/pln/rules")
(add-to-load-path "../../../opencog/pln/meta-rules")

(define rule-filenames
  (list ;; "term/deduction.scm"
        "predicate/conditional-full-instantiation.scm"
        "propositional/fuzzy-conjunction-introduction.scm"
        "propositional/contraposition.scm"
  )
)
(for-each load-from-path rule-filenames)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; List the rules
(define rules
  (list ;; deduction-inheritance-rule-name
        conditional-full-instantiation-implication-scope-meta-rule-name
        conditional-full-instantiation-implication-meta-rule-name
        conditional-full-instantiation-inheritance-meta-rule-name
        fuzzy-conjunction-introduction-3ary-rule-name
        crisp-contraposition-implication-scope-rule-name
  )
)

;; Associate rules to PLN
(ure-add-rules pln-rbs rules)

;; ;;;;;;;;;;;;;;;;;;;;;;
;; ;; Other parameters ;;
;; ;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters
(ure-set-num-parameter pln-rbs "URE:maximum-iterations" 500)

;; Attention allocation (0 to disable it, 1 to enable it)
(ure-set-fuzzy-bool-parameter pln-rbs "URE:attention-allocation" 0)

;; Complexity penalty
(ure-set-num-parameter pln-rbs "URE:complexity-penalty" 1)

;; BIT reduction parameters
(ure-set-num-parameter pln-rbs "URE:BC:maximum-bit-size" 50000)
