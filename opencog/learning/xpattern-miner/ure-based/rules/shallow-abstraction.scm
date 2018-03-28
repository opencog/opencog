;; Rule to generate shallow abstractions of a given pattern,
;; specifically
;;
;; Evaluation
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       VariableList
;;         <x1>
;;         ...
;;         <xn>
;;       <g-body>
;;     <ms>
;; |-
;; Set
;;   Evaluation (stv 1 1)
;;     Predicate "shabs"
;;     List
;;       List
;;         <f1>
;;         <x2>
;;         ...
;;         <xn>
;;     Evaluation
;;       Predicate "minsup"
;;       List
;;         Lambda
;;           VariableList
;;             <x1>
;;             ...
;;             <xn>
;;           <g-body>
;;         <ms>
;; ...
;;
;; where f1 to fn are a shallow abstractions (shabs stands for shallow
;; abstraction) either functions, constant nodes, or a variable nodes
;; amongst x1 to xn.
;;
;; Note, it's weird that the second argument of shabs is the minsup
;; evaluation, because it might be equivalent to it's TV, it's not
;; clear though so we'll go with that for now.



;; TODO

;; Function to generate shallow abstractions
(define (gen-shallow-abstraction link-type arity)
  (let* ((variables (gen-variables "$sha-arg" arity))
         (vardecl (VariableList variables))
         (body (link-type variables)))
      (Lambda
        vardecl
        (if (equal? (cog-type body) 'AndLink)
            (LocalQuote body)
            body))))

;; (define inheritance-shallow-abstraction
;;   (gen-shallow-abstraction InheritanceLink 2))

;; (define and-shallow-abstraction
;;   (gen-shallow-abstraction AndLink 2))
