;;
;; URE Configuration file for a mere fuzzy conjunction rule
;;
;; Before running any PLN inference you must load that file in the
;; AtomSpace

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Load required modules and utils ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(use-modules (opencog))
(use-modules (opencog rule-engine))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define a rule-based system ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define propositional-rule-base (ConceptNode "propositional-rule-base"))
(InheritanceLink
   propositional-rule-base
   (ConceptNode "URE")
)

;; Define conj-bc for convenience
(define (prop-bc target) (cog-bc propositional-rule-base target (List) (Set)))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules of the propositional rule base
(define pln-rules-dir "../../../opencog/pln/rules/")
(define (append-rule-dir basename) (string-append pln-rules-dir basename))
(define rule-basenames
  (list "fuzzy-conjunction-introduction.scm"
        "fuzzy-disjunction-introduction.scm"
        "negation-introduction.scm"
        )
  )
(define rule-files (map append-rule-dir rule-basenames))
(for-each load rule-files)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to the conjunction rule base ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; List the rules and their weights.
(define rules
  (list
        (list fuzzy-conjunction-introduction-1ary-rule-name 1)
        (list fuzzy-conjunction-introduction-2ary-rule-name 1)
        (list fuzzy-conjunction-introduction-3ary-rule-name 1)
        (list fuzzy-conjunction-introduction-4ary-rule-name 1)
        (list fuzzy-conjunction-introduction-5ary-rule-name 1)
        (list fuzzy-disjunction-introduction-1ary-rule-name 1)
        (list fuzzy-disjunction-introduction-2ary-rule-name 1)
        (list fuzzy-disjunction-introduction-3ary-rule-name 1)
        (list fuzzy-disjunction-introduction-4ary-rule-name 1)
        (list fuzzy-disjunction-introduction-5ary-rule-name 1)
        (list negation-introduction-rule-name 1)
        )
  )

;; Associate rules to the conjunction rule base
(ure-add-rules propositional-rule-base rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters. We set 20 iterations, propositions
;; requiring more computation steps won't be evaluated.
(ure-set-num-parameter propositional-rule-base "URE:maximum-iterations" 20)
