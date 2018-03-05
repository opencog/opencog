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

(define (gen-shallow-abstraction link-type arity)
  (let* ((variables (gen-variables "$sha-arg" arity))
         (vardecl (VariableList variables))
         (body (link-type variables)))
      (Lambda
        vardecl
        (if (equal? (cog-type body) 'AndLink)
            (LocalQuote body)
            body))))

(define inheritance-shallow-abstraction
  (gen-shallow-abstraction InheritanceLink 2))

(define and-shallow-abstraction
  (gen-shallow-abstraction AndLink 2))
