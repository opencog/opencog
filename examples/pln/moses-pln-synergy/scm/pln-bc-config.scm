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
(use-modules (opencog rule-engine))

;XXX This is bad and broken and wrong; one should not try to bypass the
; scheme module system like this, its just asking for carpet burns.
(load-from-path "utilities.scm")
(load-from-path "av-tv.scm")
(load-from-path "opencog/rule-engine/rule-engine-utils.scm")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define PLN rule-based system ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define pln-rbs (ConceptNode "PLN"))
(InheritanceLink
   pln-rbs
   (ConceptNode "URE")
)

;; Define pln-fc and pln-bc for convenience
(define (pln-fc source) (cog-fc source pln-rbs (SetLink)))
(define (pln-bc target) (cog-bc target pln-rbs (SetLink)))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules. Either w.r.t this file path
(add-to-load-path "../../../../opencog/pln/rules/")
(add-to-load-path "../../../../opencog/pln/meta-rules/")
;; Or the corresponding unit test
(add-to-load-path "../../../opencog/pln/rules/")

(define rule-filenames
  (list "conditional-partial-instantiation-meta-rule.scm"
        "implication-instantiation-rule.scm"
        "implication-scope-distribution-rule.scm"
        "and-lambda-distribution-rule.scm"
        "lambda-grounded-construction-rule.scm"
        "implication-construction-rule.scm"
        "implication-implicant-distribution-rule.scm"
        "implication-and-lambda-factorization-rule.scm"
        "deduction-rule.scm"
        "equivalence-to-implication-rule.scm"
        "implication-implicant-disjunction-rule.scm"
        )
  )
(for-each load-from-path rule-filenames)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; List the rules and their weights.
(define rules
  (list
        (list conditional-partial-instantiation-meta-rule-name 1)
        ;; (list implication-partial-instantiation-rule-name 1)
        (list implication-scope-distribution-rule-name 1)
        ;; (list and-lambda-distribution-rule-name 1)
        ;; (list lambda-grounded-construction-rule-name 1)
        ;; (list implication-construction-rule-name 1)
        ;; (list implication-implicant-distribution-rule-name 1)
        ;; (list implication-and-lambda-factorization-rule-name 1)
        ;; (list deduction-implication-rule-name 1)
        ;; (list implication-full-instantiation-rule-name 1)
        ;; (list equivalence-to-implication-rule-name 1)
        ;; (list implication-implicant-disjunction-rule-name 1)
        )
  )

;; Associate rules to PLN
(ure-add-rules pln-rbs rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters
(ure-set-num-parameter pln-rbs "URE:maximum-iterations" 10)

;; Attention allocation (0 to disable it, 1 to enable it)
(ure-set-fuzzy-bool-parameter pln-rbs "URE:attention-allocation" 0)
