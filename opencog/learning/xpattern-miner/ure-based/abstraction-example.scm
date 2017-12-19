;; This take rule fetch inheritance links, and replace the second
;; argument by a variable.
;;
;; Counts are ignored for now.
(define abstraction-example-rule
  (BindLink
    ;; Specialized pattern
    (Quote
      (Lambda
        (Variable "$X")
        (Inheritance
          (Unquote (Variable "$inh-arg-1"))
          (Unquote (Variable "$inh-arg-2")))))
    ;; Abstract pattern
    (Quote
      (Lambda
        (VariableList
          (Variable "$X")
          (Variable "$Y"))
        (Inheritance
          (Unquote (Variable "$inh-arg-1"))
          (Variable "$Y"))))))
