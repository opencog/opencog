;; URE Configuration file for producing Inference Control Rules

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Load required modules and utils ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(use-modules (opencog))
(use-modules (opencog rule-engine))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define inference control rule-base system ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Define icr-bc to evaluation inference control rule implication
;; scope link based on evidence
(define icr-rb (ConceptNode "icr-rb"))
(define (icr-bc . args)
  (apply cog-bc (cons icr-rb args)))

;; Define pre-processing rule-base to produce antecedants then used by
;; icr-rb
(define pp-icr-rb (ConceptNode "pp-icr-rb"))
(define (pp-icr-bc . args)
  (apply cog-bc (cons pp-icr-rb args)))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules. Either w.r.t this file path
(add-to-load-path "../../../opencog/pln/")

(define rule-filenames
  (list "rules/predicate/conditional-direct-evaluation.scm"
        "rules/crisp/propositional/true-conjunction-introduction.scm"
        )
  )
(for-each load-from-path rule-filenames)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to rule bases ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ure-add-rule icr-rb conditional-direct-evaluation-implication-scope-rule-name)
(ure-add-rule pp-icr-rb true-conjunction-introduction-2ary-rule-name)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

(ure-set-num-parameter icr-rb "URE:maximum-iterations" 2)
(ure-set-num-parameter pp-icr-rb "URE:maximum-iterations" 2)
