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
(define (pln-fc source) (cog-fc pln-rbs source))
(define (pln-bc target) (cog-bc pln-rbs target))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules. Either w.r.t this file path
(add-to-load-path "../../../../opencog/pln/rules/")
(add-to-load-path "../../../../opencog/pln/meta-rules/")
;; Or the corresponding unit test
(add-to-load-path "../../../opencog/pln/rules/")

(define rule-filenames
  (list "predicate/conditional-partial-instantiation.scm"
        "wip/implication-scope-to-implication.scm"
        "wip/and-lambda-distribution.scm"
        "wip/closed-lambda-introduction.scm"
        "wip/implication-introduction.scm"
        "wip/implication-implicant-distribution.scm"
        "wip/implication-and-lambda-factorization.scm"
        "term/deduction.scm"
        "wip/equivalence-to-implication.scm"
        "wip/implication-implicant-disjunction.scm"
        "predicate/conditional-full-instantiation.scm"
        )
  )
(for-each load-from-path rule-filenames)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; List the rules
(define rules
  (list
        conditional-partial-instantiation-meta-rule-name
        implication-scope-to-implication-rule-name
        ;; and-lambda-distribution-rule-name
        closed-lambda-introduction-rule-name
        implication-introduction-rule-name
        implication-implicant-distribution-rule-name
        implication-and-lambda-factorization-rule-name
        deduction-implication-rule-name
        conditional-full-instantiation-meta-rule-name
        ;; implication-full-instantiation-rule-name
        equivalence-to-implication-rule-name
        ;; implication-implicant-disjunction-rule-name
        )
  )

;; Associate rules to PLN
(ure-add-rules pln-rbs rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters
(ure-set-num-parameter pln-rbs "URE:maximum-iterations" 200000)

;; Attention allocation (0 to disable it, 1 to enable it)
(ure-set-fuzzy-bool-parameter pln-rbs "URE:attention-allocation" 0)

;; Complexity penalty
(ure-set-num-parameter pln-rbs "URE:BC:complexity-penalty" 1)

;; BIT reduction parameters
(ure-set-num-parameter pln-rbs "URE:BC:maximum-bit-size" 100000)
