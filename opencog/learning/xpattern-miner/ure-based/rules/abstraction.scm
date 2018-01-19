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
(define (gen-abstraction-rule link-type arity)
  (let* ((variables (gen-variables "$X" arity))
         (new-variables (gen-rand-variables "$X" 16 16 arity))
         (pattern (link-type variables)))
    (Bind
      (VariableList variables)
      (link-type variables)
      (Lambda
        (VariableList new-variables)
        (link-type new-variables)))))

(define inheritance-abstraction-rule (gen-abstraction-rule InheritanceLink 2))
