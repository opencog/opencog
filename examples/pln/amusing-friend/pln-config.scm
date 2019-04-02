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

;; TODO Maybe we can remove that definitely?
;; ;XXX This is bad and broken and wrong; one should not try to bypass the
;; ; scheme module system like this, its just asking for carpet burns.
;; (load-from-path "utilities.scm")
;; (load-from-path "av-tv.scm")
;; (load-from-path "opencog/ure/ure-utils.scm")

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

;; Load the rules (use load for relative path w.r.t. to that file)
(define pln-rules-dir "../../../opencog/pln/rules/")
(define (append-rule-dir basename) (string-append pln-rules-dir basename))
(define rule-basenames
  (list "wip/implication-instantiation.scm"
        "wip/implication-scope-to-implication.scm"
        "wip/equivalence-to-implication.scm"
        "wip/predicate-lambda-evaluation.scm"
        "wip/inversion.scm"
        "wip/implication-implicant-conjunction.scm"
        "wip/and-lambda-factorization-double-implication.scm"
        "term/deduction.scm"
        "wip/implication-to-implication-scope.scm"
        "wip/equivalence-scope-distribution.scm"
        "wip/and-introduction.scm"
        )
  )
(define rule-files (map append-rule-dir rule-basenames))
(for-each load rule-files)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; List the rules
(define rules
  (list scope-to-implication-rule-name
        full-instantiation-rule-name
        equivalence-to-implication-rule-name
        lambda-predicate-evaluation-rule-name
        inversion-implication-rule-name
        implication-implicant-conjunction-rule-name
        and-lambda-factorization-double-implication-rule-name
        deduction-implication-rule-name
        implication-to-implication-scope-rule-name
        equivalence-scope-distribution-rule-name
        and-introduction-grounded-evaluation-rule-name
        )
  )

;; Associate rules to PLN
(ure-add-rules pln-rbs rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters
(ure-set-num-parameter pln-rbs "URE:maximum-iterations" 50000)

;; Attention allocation (0 to disable it, 1 to enable it)
(ure-set-fuzzy-bool-parameter pln-rbs "URE:attention-allocation" 0)
