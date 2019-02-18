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

(define ppc-rbs (ConceptNode "post-process-corpus-rule-base"))

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

;; List the rules
(define rules
  (list
     conditional-full-instantiation-implication-scope-meta-rule-name
     fuzzy-conjunction-introduction-2ary-rule-name
  )
)

;; Associate rules to ppc
(ure-add-rules ppc-rbs rules)

;;;;;;;;;;;;;;;;;;;;;;
;; Other parameters ;;
;;;;;;;;;;;;;;;;;;;;;;

;; Termination criteria parameters
(ure-set-num-parameter ppc-rbs "URE:maximum-iterations" 100)
