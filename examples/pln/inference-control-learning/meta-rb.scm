;; URE Configuration file for inferring Inference Control Rules

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Load required modules and utils ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(use-modules (opencog))
(use-modules (opencog rule-engine))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define post-process corpus rule-base system ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TODO

(define pln-rbs (ConceptNode "PLN"))
(InheritanceLink
   pln-rbs
   (ConceptNode "URE")
)

;; Define pln-bc for convenience
(define* (pln-bc target #:key (trace-as #f))
  (cog-bc pln-rbs target #:trace-as trace-as))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules. Either w.r.t this file path
(add-to-load-path "../../../opencog/pln/rules/")
(add-to-load-path "../../../opencog/pln/meta-rules/")

;; TODO: add as many valid rules are possible
(define rule-filenames
  (list "term/deduction.scm"
        )
  )
(for-each load-from-path rule-filenames)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TODO: add as many valid rules are possible
; List the rules and their weights.
(define rules
  (list
        (list deduction-inheritance-rule-name 1)
        )
  )

;; Associate rules to PLN
(ure-add-rules pln-rbs rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters
(ure-set-num-parameter pln-rbs "URE:maximum-iterations" 20)

;; Attention allocation (0 to disable it, 1 to enable it)
(ure-set-fuzzy-bool-parameter pln-rbs "URE:attention-allocation" 0)

;; Complexity penalty
(ure-set-num-parameter pln-rbs "URE:BC:complexity-penalty" 1)

;; BIT reduction parameters
(ure-set-num-parameter pln-rbs "URE:BC:maximum-bit-size" 100000)
