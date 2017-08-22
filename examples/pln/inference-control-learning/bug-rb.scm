;; URE Configuration file for post-processing inference traces to
;; produce an inference history corpus.

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Load required modules and utils ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(use-modules (opencog))
(use-modules (opencog rule-engine))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define post-process corpus rule-base system ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define bug-rbs (ConceptNode "bug-rule-base"))
(InheritanceLink
   bug-rbs
   (ConceptNode "URE"))

;; Define bug-bc for convenience
(define (bug-bc . args)
  (apply cog-bc (cons bug-rbs args)))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

;; Load the rules. Either w.r.t this file path
(add-to-load-path "../../../opencog/pln/")

(define rule-filenames
  (list "meta-rules/predicate/conditional-full-instantiation.scm"
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
  )
)

;; Associate rules to bug
(ure-add-rules bug-rbs rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters
(ure-set-num-parameter bug-rbs "URE:maximum-iterations" 1)
