;
; URE Configuration file for PLN
;
; Before running any PLN inference you must load that file in the
; AtomSpace
;
; In order to add new rules you need to hack this file in 2 places
;
; 1. In the Load rules section, to add the file name where the rule is
; defined (see define rule-files).
;
; 2. In the Associate rules to PLN section, to add the name of the
; rule and its weight (see define rules).

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Load required modules and utils ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(use-modules (opencog))
(use-modules (opencog rule-engine))

(load-from-path "utilities.scm")
(load-from-path "av-tv.scm")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Define PLN rule-based system ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define pln-rbs (ConceptNode "PLN"))
(InheritanceLink
   pln-rbs
   (ConceptNode "URE")
)

; Define pln-fc and pln-bc for convenience 
(define (pln-fc source) (cog-fc source pln-rbs))
(define (pln-bc target) (cog-bc target pln-rbs))

;;;;;;;;;;;;;;;;
;; Load rules ;;
;;;;;;;;;;;;;;;;

; Load the rules (use load for relative path w.r.t. to that file)
(define rule-files (list "rules/deduction.scm"
                         "rules/modus-ponens.scm"))
(for-each load rule-files)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Associate rules to PLN ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; List the rules and their weights.
(define rules (list (list pln-rule-deduction 1)
                    (list pln-rule-modus-ponens 1))
)

; Given a pair (rule weight) create
;
; MemberLink (stv weight 1)
;    rule
;    (ConceptNode "PLN")
(define (add-rule-to-pln weighted-rule)
  (MemberLink (stv (cadr weighted-rule) 1)
     (car weighted-rule)
     pln-rbs)
)

; Associate rules to PLN
(for-each add-rule-to-pln rules)

;;;;;;;;;;;;;;;;;;;;;
;; Other paramters ;;
;;;;;;;;;;;;;;;;;;;;;

; Helper to set numerical parameters. Given a parameter name and its
; value create
;
; ExecutionLink
;    SchemaNode name
;    (ConceptNode "PLN")
;    (NumberNode value)
;
; It will also delete the any
;
; ExecutionLink
;    SchemaNode name
;    (ConceptNode "PLN")
;    *
;
; to be sure there is ever only one value associated to a parameter.
;
; TODO: fix the deletion of previous ExecutionLink
(define (set-pln-num-parameter name value)
  ;; ; Delete any previous parameter
  ;; (cog-bind (BindLink
  ;;              (ExecutionLink
  ;;                 (SchemaNode name)
  ;;                 pln-rbs
  ;;                 (VariableNode "__VALUE__"))
  ;;              (DeleteLink
  ;;                 (ExecutionLink
  ;;                    (SchemaNode name)
  ;;                    pln-rbs
  ;;                    (VariableNode "__VALUE__")))))
  ; Set new value for that parameter
  (ExecutionLink
     (SchemaNode name)
     pln-rbs
     (NumberNode (number->string value)))
)

; Helper to set (fuzzy) bool parameters. Given a parameter name and
; its value create (or overwrite)
;
; EvaluationLink (stv value 1)
;    PredicateNode name
;    (ConceptNode "PLN")
(define (set-pln-fuzzy-bool-parameter name value)
  (EvaluationLink (stv value 1)
     (SchemaNode name)
     pln-rbs)
)

; Termination criteria parameters
(set-pln-num-parameter "URE:maximum-iterations" 20)

; Attention allocation (0 to disable it, 1 to enable it)
(set-pln-fuzzy-bool-parameter "URE:attention-allocation" 0)
