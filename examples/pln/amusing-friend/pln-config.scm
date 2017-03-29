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

;; TODO Maybe we can remove that definitely?
;; ;XXX This is bad and broken and wrong; one should not try to bypass the
;; ; scheme module system like this, its just asking for carpet burns.
;; (load-from-path "utilities.scm")
;; (load-from-path "av-tv.scm")
;; (load-from-path "opencog/rule-engine/rule-engine-utils.scm")

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

;; Load the rules (use load for relative path w.r.t. to that file)
(define pln-rules-dir "../../../opencog/pln/rules/")
(define (append-rule-dir basename) (string-append pln-rules-dir basename))
(define rule-basenames
  (list "implication-instantiation-rule.scm"
        "implication-scope-to-implication-rule.scm"
        "equivalence-to-implication-rule.scm"
        "predicate-lambda-evaluation-rule.scm"
        "inversion-rule.scm"
        "implication-implicant-conjunction-rule.scm"
        "and-lambda-factorization-double-implication-rule.scm"
        "deduction-rule.scm"
        "implication-to-implication-scope-rule.scm"
        "equivalence-scope-distribution-rule.scm"
        "and-introduction-rule.scm"
        )
  )
(define rule-files (map append-rule-dir rule-basenames))
(for-each load rule-files)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; List the rules and their weights.
(define rules
  (list (list implication-scope-to-implication-rule-name 1)
        (list implication-full-instantiation-rule-name 1)
        (list equivalence-to-implication-rule-name 1)
        (list lambda-predicate-evaluation-rule-name 1)
        (list inversion-implication-rule-name 1)
        (list implication-implicant-conjunction-rule-name 1)
        (list and-lambda-factorization-double-implication-rule-name 1)
        (list deduction-implication-rule-name 1)
        (list implication-to-implication-scope-rule-name 1)
        (list equivalence-scope-distribution-rule-name 1)
        (list and-introduction-grounded-evaluation-rule-name 1)
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
