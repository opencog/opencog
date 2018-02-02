;; Given a ground atom, abstract it
;;
;; L
;;   A1
;;   ...
;;   An
;; |-
;; Lambda
;;   V1
;;   ...
;;   Vn
;;   L
;;     V1
;;     ...
;;     Vn
;;
;; V1 to Vn are random generated variables.

;; TODO: these are in fact, facts!

;; TODO: rather build a rule to create some thing like
;;
;; Evaluation
;;   Predicate "shallow-abstraction-of"
;;   List
;;     <shallow-pattern>
;;     <valuations>

(define (gen-abstraction-rule link-type arity)
  (let* ((variables (gen-variables "$X" arity))
         (new-variables (gen-rand-variables "$X" 16 16 arity))
         (pattern (link-type variables)))
    ;; Actually since the conclusion is a close term, we may directly
    ;; produce the conclusion without the rule.
    ;; (Bind
    ;;   (VariableList variables)
    ;;   (link-type variables)
      (Lambda
        (VariableList new-variables)
        (link-type new-variables))))

(define inheritance-abstraction-rule (gen-abstraction-rule InheritanceLink 2))
