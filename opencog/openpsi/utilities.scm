;
; utilities.scm
; Helper functions for OpenPsi
;
; Copyright (C) 2015-2016 OpenCog Foundation
; Copyright (C) 2017 MindCloud

(use-modules (ice-9 regex)) ; For string-match
(use-modules (srfi srfi-1)) ; For fold, delete-duplicates

(use-modules (opencog) (opencog exec))

; --------------------------------------------------------------
; XXX TODO: does this really need to be public? change into atom.
(define psi-prefix-str "OpenPsi: ")

; --------------------------------------------------------------
; XXX TODO: does this really need to be public?
(define (psi-suffix-str a-string)
"
  psi-suffix-str STRING

  Given the string STRING, this removes the psi prefix string.
"
    (let ((z-match (string-match psi-prefix-str a-string)))
        (if z-match
            (match:suffix z-match)
            (error (string-append "The string argument must have the prefix: "
                "\"" psi-prefix-str "\". " "Instead got:" a-string) )
        )
    )
)

; --------------------------------------------------------------
(define (psi-set-func! function is-eval component-node function-name)
"
  psi-set-func! FUNC IS-EVAL COMPONENT FUNC-NAME

  Associate a function with a particular component.

  FUNC is an atom that can be executed or evaluated. It will perform
    the function for the particular component.

  IS-EVAL is a the string '#t' if the function is evaluatable and '#f' if
    it is executable.

  COMPONENT should be a component node that the function will
    be assocaited with.

  FUNC-NAME is the type of function.
"
  ; Record whether the function is evaluatable or executable.
  (cog-set-value!
    function
    (Predicate "is_evaluatable?")
    (StringValue is-eval))

  ; Record the function with the component-node used to represent it.
  (cog-set-value!
    component-node
    (Predicate function-name)
    function)
)

; --------------------------------------------------------------
(define (psi-func component-node function-name)
"
  psi-func COMPONENT FUNC-NAME

  Return the node that represents the function for the given component
  or nil if it doesn't exist.

  COMPONENT should be a component node that the function is set for.

  FUNC-NAME should be the type of function.
"
  (cog-value component-node (Predicate function-name))
)

; --------------------------------------------------------------
(define (psi-func-evaluatable? component-node function-name)
"
  psi-func-evaluatable? COMPONENT FUNC-NAME

  Return '#t' if the function is evaluatable and '#f' if
    it is executable.

  COMPONENT should be a component node that the function is set for.

  FUNC-NAME should be the type of function.
"
  (let ((func (psi-func component-node function-name)))
    (if (null? func)
      (error (format #f "A function called \"~a\" hasn't been set for ~a\n"
          function-name component-node))
      (equal? "#t"
        (cog-value-ref (cog-value func (Predicate "is_evaluatable?")) 0))
    )
  )
)

; --------------------------------------------------------------
