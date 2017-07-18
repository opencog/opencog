;; URE Configuration file for inferring Inference Control Rules

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Load required modules and utils ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(use-modules (opencog))
(use-modules (opencog rule-engine))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define post-process corpus rule-base system ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define ppc-rbs (ConceptNode "post-process-corpus-rule-base"))
(InheritanceLink
   ppc-rbs
   (ConceptNode "URE"))

;; Define ppc-bc for convenience
(define (ppc-bc . args)
  (apply cog-bc (cons ppc-rbs args)))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules. Either w.r.t this file path
(add-to-load-path "../../../opencog/pln/")

(define rule-filenames
  (list "meta-rules/predicate/conditional-full-instantiation.scm"
        "rules/propositional/fuzzy-conjunction-introduction.scm"
        )
  )
(for-each load-from-path rule-filenames)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; List the rules and their weights.
(define rules
  (list
     (list conditional-full-instantiation-implication-scope-meta-rule-name 1)
     (list conditional-full-instantiation-implication-meta-rule-name 1)
     (list fuzzy-conjunction-introduction-2ary-rule-name 1)
  )
)

;; Associate rules to ppc
(ure-add-rules ppc-rbs rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters
(ure-set-num-parameter ppc-rbs "URE:maximum-iterations" 50)

;; Attention allocation (0 to disable it, 1 to enable it)
(ure-set-fuzzy-bool-parameter ppc-rbs "URE:attention-allocation" 0)

;; Complexity penalty
(ure-set-num-parameter ppc-rbs "URE:BC:complexity-penalty" 1)

;; BIT reduction parameters
(ure-set-num-parameter ppc-rbs "URE:BC:maximum-bit-size" 100000)
