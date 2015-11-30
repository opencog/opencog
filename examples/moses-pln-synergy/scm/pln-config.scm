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

;; Define pln-fc and pln-bc for convenience 
(define (pln-fc source) (cog-fc source pln-rbs))
(define (pln-bc target) (cog-bc target pln-rbs (SetLink)))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules (use load for relative path w.r.t. to that file)
(define pln-rules-dir "../../../opencog/reasoning/pln/rules/")
(define rule-files
  (list (string-append pln-rules-dir "deduction-rule.scm")
        (string-append pln-rules-dir "modus-ponens-rule.scm")
        (string-append pln-rules-dir "implication-instantiation-rule.scm")
        (string-append pln-rules-dir "implication-lambda-distribution-rule.scm")
        (string-append pln-rules-dir "and-lambda-distribution-rule.scm")
        (string-append pln-rules-dir "implication-and-lambda-factorization-rule.scm")
        (string-append pln-rules-dir "lambda-grounded-construction-rule.scm")
        (string-append pln-rules-dir "implication-construction-rule.scm")
        (string-append pln-rules-dir "implication-implicant-distribution-rule.scm")
        (string-append pln-rules-dir "equivalence-to-double-implication-rule.scm")
        "pln-rules/implication-or.scm"
        )
)
(for-each load rule-files)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; ; List the rules and their weights.
;; (define rules (list (list deduction-implication-rule-name 1)
;;                     (list modus-ponens-implication-rule-name 1)
;;               )
;; )

;; ; Associate rules to PLN
;; (ure-add-rules pln-rbs rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters
(ure-set-num-parameter pln-rbs "URE:maximum-iterations" 20)

;; Attention allocation (0 to disable it, 1 to enable it)
(ure-set-fuzzy-bool-parameter pln-rbs "URE:attention-allocation" 0)
