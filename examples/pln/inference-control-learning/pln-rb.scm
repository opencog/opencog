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
(define (pln-bc . args)
  (apply cog-bc (cons pln-rbs args)))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules. Either w.r.t this file path
(add-to-load-path "../../../opencog/pln/")

;; TODO: add more rules
(define rule-filenames
  (list "rules/propositional/modus-ponens.scm"
        "rules/propositional/contraposition.scm"
        "rules/term/deduction.scm"
        "meta-rules/predicate/conditional-full-instantiation.scm"
        )
  )
(for-each load-from-path rule-filenames)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TODO: add more rules
; List the rules
(define rules
  (list
        modus-ponens-inheritance-rule-name
        modus-ponens-implication-rule-name
        modus-ponens-subset-rule-name
        contraposition-inheritance-rule-name
        contraposition-implication-rule-name
        crisp-contraposition-implication-scope-rule-name
        deduction-inheritance-rule-name
        deduction-implication-rule-name
        deduction-subset-rule-name
        conditional-full-instantiation-implication-scope-meta-rule-name
        conditional-full-instantiation-implication-meta-rule-name
        conditional-full-instantiation-inheritance-meta-rule-name
        )
  )

;; Associate rules to PLN
(ure-add-rules pln-rbs rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters
(ure-set-num-parameter pln-rbs "URE:maximum-iterations" piter)

;; Complexity penalty
(ure-set-num-parameter pln-rbs "URE:complexity-penalty" 0.1)

;; Mixture Model compressiveness
(ure-set-num-parameter pln-rbs "URE:BC:MM:compressiveness" 0.5)

;; Mixture Model complexity penalty
(ure-set-num-parameter pln-rbs "URE:BC:MM:complexity-penalty" 0.5)
