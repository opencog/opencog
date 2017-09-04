;; URE Configuration file for producing Inference Control Rules

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Load required modules and utils ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(use-modules (opencog))
(use-modules (opencog rule-engine))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define post-process corpus rule-base system ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define icr-rbs (ConceptNode "inference-control-rules-rule-base"))
(InheritanceLink
   icr-rbs
   (ConceptNode "URE"))

;; Define icr-bc for convenience
(define (icr-bc . args)
  (apply cog-bc (cons icr-rbs args)))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules. Either w.r.t this file path
(add-to-load-path "../../../opencog/pln/")

(define rule-filenames
  (list "rules/predicate/conditional-direct-evaluation.scm"
        )
  )
(for-each load-from-path rule-filenames)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; List the rules
(define rules
  (list conditional-direct-evaluation-implication-scope-rule-name)
)

;; Associate rules to ppc
(ure-add-rules icr-rbs rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters
(ure-set-num-parameter icr-rbs "URE:maximum-iterations" 100)

;; Attention allocation (0 to disable it, 1 to enable it)
(ure-set-fuzzy-bool-parameter icr-rbs "URE:attention-allocation" 0)

;; Complexity penalty
(ure-set-num-parameter icr-rbs "URE:BC:complexity-penalty" 1)

;; BIT reduction parameters
(ure-set-num-parameter icr-rbs "URE:BC:maximum-bit-size" 100000)
