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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define PLN rule-based system ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define pln-rbs (ConceptNode "PLN"))
(InheritanceLink
   pln-rbs
   (ConceptNode "URE")
)

;; Define pln-bc for convenience
(define* (pln-bc . args)
  (apply cog-bc (cons pln-rbs args)))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules. Either w.r.t this file path
(add-to-load-path "../../../opencog/pln/rules/")
(add-to-load-path "../../../opencog/pln/meta-rules/")

;; TODO: add more rules
(define rule-filenames
  (list "propositional/modus-ponens.scm"
        ;; "propositional/contraposition.scm"
        ;; "term/deduction.scm"
        )
  )
(for-each load-from-path rule-filenames)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TODO: add more rules
; List the rules and their weights.
(define rules
  (list
        (list modus-ponens-inheritance-rule-name 1)
        ;; (list modus-ponens-implication-rule-name 1)
        ;; (list modus-ponens-subset-rule-name 1)
        ;; (list contraposition-inheritance-rule-name 1)
        ;; (list contraposition-implication-rule-name 1)
        ;; (list crisp-contraposition-implication-scope-rule-name 1)
        ;; (list deduction-inheritance-rule-name 1)
        ;; (list deduction-implication-rule-name 1)
        ;; (list deduction-subset-rule-name 1)
        )
  )

;; Associate rules to PLN
(ure-add-rules pln-rbs rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters
(ure-set-num-parameter pln-rbs "URE:maximum-iterations" 1)

;; ;; Attention allocation (0 to disable it, 1 to enable it)
;; (ure-set-fuzzy-bool-parameter pln-rbs "URE:attention-allocation" 0)

;; ;; Complexity penalty
;; (ure-set-num-parameter pln-rbs "URE:BC:complexity-penalty" 1)

;; ;; BIT reduction parameters
;; (ure-set-num-parameter pln-rbs "URE:BC:maximum-bit-size" 100000)
