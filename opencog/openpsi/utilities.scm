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
    (if (nil? func)
      (error (format #f "A function called \"~a\" hasn't been set for ~a\n"
          function-name component-node))
      (equal? "#t"
        (cog-value-ref (cog-value func (Predicate "is_evaluatable?")) 0))
    )
  )
)

; --------------------------------------------------------------
; Utilites for openpsi parameters. These parameters are meant to be
; used for defineing modulators, demanads ....
(define value-key (Predicate "psi-param-value"))

(define (set-value! atom num)
  (cog-set-value! atom value-key (FloatValue num))
)

(define (psi-param name)
"
  psi-param NAME

  Returns a (ConceptNode NAME) that represents an openpsi parameter.
"
  (Concept name)
)

(define (psi-param-value atom)
"
  psi-param-value ATOM

  Returns the value of the parameter represented by ATOM.
"
  (define v (cog-value atom value-key))
  (if (nil? v)
    (begin (set-value! atom 0) 0)
    (cog-value-ref v 0))
)

(define (calc-inc-value atom num)
  (+ (psi-param-value atom) (abs num))
)

(define (psi-param-increase! atom num)
"
  psi-param-increase! ATOM NUM

  Returns ATOM after increasing the value of the parameter represented by it,
  by an amount equal to the magnitude of NUM. The maximum amount the value
  is increased to is 1.
"
  (let ((v (calc-inc-value atom num)))
    (if (> 1 v)
      (set-value! atom v)
      (set-value! atom 1)))
)

(define (calc-dec-value atom num)
  (- (psi-param-value atom) (abs num))
)

(define (psi-param-decrease! atom num)
"
  psi-param-decrease! ATOM NUM

  Returns ATOM after decreasing the value of the parameter represented by it,
  by an amount equal to the magnitude of NUM. The minimum amout the value
  is decreased to is -1.
"
  (let ((v (- (psi-param-value atom) (abs num))))
    (if (< -1 v)
      (set-value! atom v)
      (set-value! atom -1)))
)

(define (psi-param-neutralize! atom num)
"
  psi-param-neutralize! ATOM NUM

  Returns ATOM after increasing/decreasing the value of the parameter
  represented by it, by an amount equal to the magnitude of NUM until it
  reaches zero.
"
  (let ((v (psi-param-value atom)))
    (cond
      ((equal? = 0) 0)
      ((negative? v)
         (let ((nv (calc-inc-value atom num)))
           (if (> nv 0)
             (set-value! atom 0)
             (set-value! atom nv))))
      ((positive? v)
         (let ((nv (calc-dec-value atom num)))
           (if (< nv 0)
             (set-value! atom 0)
             (set-value! atom nv))))
    ))
)

; --------------------------------------------------------------
